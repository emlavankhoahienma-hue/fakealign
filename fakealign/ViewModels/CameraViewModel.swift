import Foundation
import SwiftUI
import AVFoundation
import CoreMedia
import Combine
import UIKit
import simd
import QuartzCore

public enum AISessionState: Equatable {
    case idle
    case analyzing
    case targetPlaced(locked: Bool)
    case alignmentPerfect
    case capturing
    case done(capturedImage: UIImage)

    public static func == (lhs: AISessionState, rhs: AISessionState) -> Bool {
        switch (lhs, rhs) {
        case (.idle, .idle), (.analyzing, .analyzing), (.alignmentPerfect, .alignmentPerfect), (.capturing, .capturing):
            return true
        case (.targetPlaced(let l), .targetPlaced(let r)):
            return l == r
        case (.done, .done):
            return true
        default:
            return false
        }
    }
}

@MainActor
public final class CameraViewModel: ObservableObject {
    // MARK: - Published Properties for UI
    @Published public var sessionState: AISessionState = .idle
    @Published public var currentTargetScreenPoint: CGPoint = CGPoint(x: 0.5, y: 0.5)
    @Published public var isTargetOffscreen: Bool = false
    @Published public var offscreenDockAngleRad: Double = 0.0

    @Published public var activeCompositionRule: CompositionRuleType = .ruleOfThirds
    @Published public var currentZoom: CGFloat = 1.0
    @Published public var recommendedZoom: CGFloat = 1.0
    @Published public var autoCaptureEnabled: Bool = true
    @Published public var exposureBias: Float = 0.0

    @Published public var lastCapturedPhoto: UIImage?

    // Sub-services
    public let cameraService = CameraService.shared
    public let motionService = DeviceMotionService.shared
    public let spatialEngine = SpatialTrackingEngine.shared
    public let visionEngine = VisionFramingEngine.shared
    public let compositionCalculator = CompositionCalculator.shared

    // Alignment gate timer
    private var alignmentTimerStart: Double?
    private var lastSamplePixelBuffer: CVPixelBuffer?
    private var cancellables = Set<AnyCancellable>()

    // Camera field of view (approx 68 deg HFOV for 1x main camera)
    private let fovHorizontalRad: Double = 68.0 * .pi / 180.0
    private let fovVerticalRad: Double = 54.0 * .pi / 180.0

    public init() {
        setupSubscriptions()
    }

    // MARK: - Lifecycle
    public func startCamera() {
        cameraService.startSession()
        motionService.startUpdates()
        sessionState = .analyzing
    }

    public func stopCamera() {
        cameraService.stopSession()
        motionService.stopUpdates()
        visionEngine.stopTracking()
        spatialEngine.unlockTarget()
        sessionState = .idle
    }

    // MARK: - Subscriptions & Per-Frame Processing
    private func setupSubscriptions() {
        // Video Sample Buffer subscription
        cameraService.sampleBufferPublisher
            .receive(on: DispatchQueue.global(qos: .userInteractive))
            .sink { [weak self] sampleBuffer in
                self?.handleCameraSampleBuffer(sampleBuffer)
            }
            .store(in: &cancellables)

        // Tracked Target from Vision Engine
        visionEngine.$activeTrackedSubject
            .receive(on: DispatchQueue.main)
            .sink { [weak self] subject in
                self?.handleVisionTrackingUpdate(subject)
            }
            .store(in: &cancellables)

        // Photo captured subscription
        cameraService.photoCapturePublisher
            .receive(on: DispatchQueue.main)
            .sink { [weak self] image in
                self?.lastCapturedPhoto = image
                self?.sessionState = .done(capturedImage: image)
            }
            .store(in: &cancellables)
    }

    private func handleCameraSampleBuffer(_ sampleBuffer: CMSampleBuffer) {
        let timestamp = CMSampleBufferGetPresentationTimeStamp(sampleBuffer).seconds

        if let pixelBuffer = CMSampleBufferGetImageBuffer(sampleBuffer) {
            self.lastSamplePixelBuffer = pixelBuffer
        }

        // Pass frame to Vision Engine (throttled inside engine)
        visionEngine.processSampleBuffer(sampleBuffer, currentPredictedRay: spatialEngine.subjectWorldRay)

        // Viewfinder Latency Compensation & 3D Ray Projection
        // Latency lead: live preview time = sample timestamp - 8ms
        let livePreviewTime = timestamp - 0.008
        var qx: Double = 0, qy: Double = 0, qz: Double = 0, qw: Double = 1.0
        let hasPose = ntc_imu_interpolate_pose(livePreviewTime, &qx, &qy, &qz, &qw)

        let cameraAttitude = hasPose ? simd_quatd(real: qw, imag: simd_double3(qx, qy, qz)) : simd_quatd(real: 1.0, imag: simd_double3(0, 0, 0))

        if spatialEngine.isTrackingActive {
            let smoothedRay = spatialEngine.updateViewfinderPresentation(currentTimestamp: timestamp, focalScale: Double(currentZoom))
            let projectedPoint = spatialEngine.projectRayToCameraScreen(
                ray: smoothedRay,
                cameraAttitude: cameraAttitude,
                fovHorizontalRad: fovHorizontalRad,
                fovVerticalRad: fovVerticalRad
            )

            DispatchQueue.main.async { [weak self] in
                self?.updateScreenTarget(projectedPoint)
                self?.checkAlignmentGate(at: projectedPoint)
            }
        }
    }

    private func handleVisionTrackingUpdate(_ subject: TrackedTargetObservation?) {
        guard let subject = subject, spatialEngine.isTrackingActive else { return }

        // Compute optical bearing ray of tracked real object
        var qx: Double = 0, qy: Double = 0, qz: Double = 0, qw: Double = 1.0
        _ = ntc_imu_interpolate_pose(CACurrentMediaTime(), &qx, &qy, &qz, &qw)
        let attitude = simd_quatd(real: qw, imag: simd_double3(qx, qy, qz))

        let observedRay = spatialEngine.screenPointToWorldRay(
            point: subject.anchorPoint,
            cameraAttitude: attitude,
            fovHorizontalRad: fovHorizontalRad,
            fovVerticalRad: fovVerticalRad
        )

        // Ingest into ESKF
        spatialEngine.ingestOpticalMeasurement(
            observedSubjectRay: observedRay,
            confidence: Double(subject.confidence),
            timestamp: CACurrentMediaTime(),
            type: .confirmedContinuation
        )
    }

    // MARK: - Target Placement (Pin Target)
    public func pinTargetAndStartMotion(target: CGPoint, subjectRect: CGRect, trackedPoint: CGPoint, type: TrackedSubjectType = .foregroundObject) {
        guard let pixelBuffer = lastSamplePixelBuffer else { return }

        var qx: Double = 0, qy: Double = 0, qz: Double = 0, qw: Double = 1.0
        let now = CACurrentMediaTime()
        _ = ntc_imu_interpolate_pose(now, &qx, &qy, &qz, &qw)
        let attitude = simd_quatd(real: qw, imag: simd_double3(qx, qy, qz))

        // Convert subject anchor (real physical point) and target aim point into 3D world rays
        let subjectRay = spatialEngine.screenPointToWorldRay(
            point: trackedPoint,
            cameraAttitude: attitude,
            fovHorizontalRad: fovHorizontalRad,
            fovVerticalRad: fovVerticalRad
        )

        let guideRay = spatialEngine.screenPointToWorldRay(
            point: target,
            cameraAttitude: attitude,
            fovHorizontalRad: fovHorizontalRad,
            fovVerticalRad: fovVerticalRad
        )

        // Lock 3D spatial coordinate
        spatialEngine.lockTarget(subjectRay: subjectRay, guideRay: guideRay, timestamp: now)

        // Start Apple Vision tracking on the physical object anchor
        visionEngine.startTrackingObject(
            boundingBox: subjectRect,
            anchorPoint: trackedPoint,
            type: type,
            initialPixelBuffer: pixelBuffer
        )

        self.sessionState = .targetPlaced(locked: true)
    }

    public func userTappedSubject(_ subject: TrackedTargetObservation) {
        // Calculate optimal composition hypothesis for tapped subject
        let hypothesis = compositionCalculator.evaluateOptimalComposition(
            subjectBox: subject.boundingBox,
            subjectAnchor: subject.anchorPoint,
            gazeVector: subject.gazeVector,
            subjectType: subject.type,
            currentZoom: currentZoom
        )

        self.activeCompositionRule = hypothesis.rule
        self.recommendedZoom = hypothesis.recommendedZoom

        // Pin the aim point and start tracking anchor
        pinTargetAndStartMotion(
            target: hypothesis.aimPoint,
            subjectRect: subject.boundingBox,
            trackedPoint: subject.anchorPoint,
            type: subject.type
        )
    }

    public func resetTarget() {
        spatialEngine.unlockTarget()
        visionEngine.stopTracking()
        alignmentTimerStart = nil
        sessionState = .analyzing
    }

    // MARK: - Screen Target Positioning & Edge Docking
    private func updateScreenTarget(_ point: CGPoint) {
        let margin: CGFloat = 0.05
        let isOff = point.x < margin || point.x > (1.0 - margin) || point.y < margin || point.y > (1.0 - margin)

        self.isTargetOffscreen = isOff

        if isOff {
            // Edge docking: clamp point to screen border and calculate directional angle
            let clampedX = max(margin, min(1.0 - margin, point.x))
            let clampedY = max(margin, min(1.0 - margin, point.y))
            self.currentTargetScreenPoint = CGPoint(x: clampedX, y: clampedY)

            // Angle towards actual target from screen center (0.5, 0.5)
            let dx = point.x - 0.5
            let dy = point.y - 0.5
            self.offscreenDockAngleRad = atan2(Double(dy), Double(dx))
        } else {
            self.currentTargetScreenPoint = point
        }
    }

    // MARK: - Alignment Gate
    private func checkAlignmentGate(at point: CGPoint) {
        guard case .targetPlaced = sessionState else { return }

        // Distance between target point and center reticle (0.5, 0.5)
        let dist = hypot(point.x - 0.5, point.y - 0.5)

        if dist <= 0.04 {
            let now = CACurrentMediaTime()
            if let start = alignmentTimerStart {
                if (now - start) >= 0.120 { // 120ms stable alignment threshold
                    triggerPerfectAlignment()
                }
            } else {
                alignmentTimerStart = now
            }
        } else {
            alignmentTimerStart = nil
        }
    }

    private func triggerPerfectAlignment() {
        // Haptic feedback
        let generator = UIImpactFeedbackGenerator(style: .medium)
        generator.prepare()
        generator.impactOccurred()

        self.sessionState = .alignmentPerfect

        // Smooth zoom ramp if recommended zoom differs
        if abs(recommendedZoom - currentZoom) > 0.1 {
            cameraService.rampZoom(to: recommendedZoom, rate: 4.0)
            self.currentZoom = recommendedZoom
        }

        // Auto capture if enabled
        if autoCaptureEnabled {
            DispatchQueue.main.asyncAfter(deadline: .now() + 0.15) { [weak self] in
                self?.takePhoto()
            }
        }
    }

    public func takePhoto() {
        guard sessionState != .capturing else { return }
        sessionState = .capturing

        cameraService.capturePhoto { [weak self] result in
            DispatchQueue.main.async {
                switch result {
                case .success(let image):
                    self?.lastCapturedPhoto = image
                    self?.sessionState = .done(capturedImage: image)
                case .failure(let error):
                    print("[CameraViewModel] Photo capture failed: \(error)")
                    self?.sessionState = .targetPlaced(locked: true)
                }
            }
        }
    }

    public func setExposureBias(_ bias: Float) {
        self.exposureBias = bias
        cameraService.setExposureBias(bias)
    }

    public func setZoom(_ zoom: CGFloat) {
        self.currentZoom = zoom
        cameraService.setLinearZoom(zoom)
    }
}
