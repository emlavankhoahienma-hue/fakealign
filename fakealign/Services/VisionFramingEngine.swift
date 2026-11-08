import Foundation
import Vision
import CoreMedia
import UIKit
import Combine

public enum TrackedSubjectType: String, CaseIterable {
    case human = "Person"
    case face = "Face / Portrait"
    case animal = "Animal"
    case foregroundObject = "Object"
}

public struct TrackedTargetObservation {
    public var boundingBox: CGRect        // In UI normalized coordinates (top-left 0,0)
    public var anchorPoint: CGPoint        // In UI normalized coordinates (e.g. eye center or object center)
    public var type: TrackedSubjectType
    public var confidence: Float
    public var gazeVector: CGVector        // Vector pointing in direction subject is looking
}

/// VisionFramingEngine runs Apple Vision requests at 24 FPS with 2-Tier Re-ID
public final class VisionFramingEngine: ObservableObject {
    public static let shared = VisionFramingEngine()

    @Published public private(set) var currentDetectedSubjects: [TrackedTargetObservation] = []
    @Published public private(set) var activeTrackedSubject: TrackedTargetObservation?
    @Published public private(set) var isTrackingTarget: Bool = false
    @Published public private(set) var isReidentifying: Bool = false

    private let sequenceHandler = VNSequenceRequestHandler()
    private var trackRequest: VNTrackObjectRequest?
    private var referenceFeaturePrint: VNFeaturePrintObservation?

    private var lastTrackedBox: CGRect?
    private var lastTrackedAnchor: CGPoint?
    private var consecutiveLostFrames: Int = 0

    private var lastProcessingTime: Double = 0.0
    private var lastSaliencyTime: Double = 0.0

    private let visionQueue = DispatchQueue(label: "com.fakealign.vision.processingQueue", qos: .userInitiated)

    private init() {}

    // MARK: - Start Tracking User Selected Target
    public func startTrackingObject(boundingBox: CGRect, anchorPoint: CGPoint, type: TrackedSubjectType, initialPixelBuffer: CVPixelBuffer) {
        visionQueue.async { [weak self] in
            guard let self = self else { return }

            // Convert UI normalized coordinates (top-left) to Vision coordinates (bottom-left)
            let visionRect = self.uiRectToVisionRect(boundingBox)
            let trackObs = VNDetectedObjectObservation(boundingBox: visionRect)
            let request = VNTrackObjectRequest(detectedObjectObservation: trackObs)
            request.trackingLevel = .accurate

            self.trackRequest = request
            self.lastTrackedBox = boundingBox
            self.lastTrackedAnchor = anchorPoint
            self.consecutiveLostFrames = 0

            // Extract reference neural feature print for Tier 2 Re-ID
            self.extractFeaturePrint(from: initialPixelBuffer, cropRect: visionRect)

            let initialTarget = TrackedTargetObservation(
                boundingBox: boundingBox,
                anchorPoint: anchorPoint,
                type: type,
                confidence: 1.0,
                gazeVector: CGVector(dx: 0, dy: 0)
            )

            DispatchQueue.main.async {
                self.activeTrackedSubject = initialTarget
                self.isTrackingTarget = true
                self.isReidentifying = false
            }
        }
    }

    public func stopTracking() {
        visionQueue.async { [weak self] in
            guard let self = self else { return }
            self.trackRequest = nil
            self.referenceFeaturePrint = nil
            self.lastTrackedBox = nil
            self.lastTrackedAnchor = nil
            self.consecutiveLostFrames = 0

            DispatchQueue.main.async {
                self.activeTrackedSubject = nil
                self.isTrackingTarget = false
                self.isReidentifying = false
            }
        }
    }

    // MARK: - Main Per-Frame Vision Dispatch (Throttled to 24 FPS)
    public func processSampleBuffer(_ sampleBuffer: CMSampleBuffer, currentPredictedRay: S2Ray?) {
        let timestamp = CMSampleBufferGetPresentationTimeStamp(sampleBuffer).seconds
        // Throttle to 24 FPS (approx 41.6ms interval)
        guard timestamp - lastProcessingTime >= (1.0 / 24.5) else {
            return
        }
        lastProcessingTime = timestamp

        guard let pixelBuffer = CMSampleBufferGetImageBuffer(sampleBuffer) else { return }

        visionQueue.async { [weak self] in
            guard let self = self else { return }

            if self.isTrackingTarget, let trackReq = self.trackRequest {
                // Tracking Pipeline
                self.performObjectTracking(pixelBuffer: pixelBuffer, request: trackReq, timestamp: timestamp, predictedRay: currentPredictedRay)
            } else {
                // Scene Understanding & Candidate Detection Pipeline
                self.detectSubjects(pixelBuffer: pixelBuffer)
            }
        }
    }

    // MARK: - Object Tracking Step
    private func performObjectTracking(pixelBuffer: CVPixelBuffer, request: VNTrackObjectRequest, timestamp: Double, predictedRay: S2Ray?) {
        do {
            try sequenceHandler.perform([request], on: pixelBuffer, orientation: .up)

            guard let results = request.results as? [VNDetectedObjectObservation],
                  let observation = results.first,
                  observation.confidence >= 0.35 else {
                handleTrackingLoss(pixelBuffer: pixelBuffer, timestamp: timestamp, predictedRay: predictedRay)
                return
            }

            // Tracking Successful
            consecutiveLostFrames = 0
            let uiBox = visionRectToUIRect(observation.boundingBox)

            // Refine anchor point: check if face landmarks are available inside tracking box
            let (refinedAnchor, gazeVec) = detectEyeAnchorOrCenter(pixelBuffer: pixelBuffer, insideBox: observation.boundingBox)

            let trackedTarget = TrackedTargetObservation(
                boundingBox: uiBox,
                anchorPoint: refinedAnchor,
                type: (refinedAnchor != CGPoint(x: uiBox.midX, y: uiBox.midY)) ? .face : .human,
                confidence: observation.confidence,
                gazeVector: gazeVec
            )

            lastTrackedBox = uiBox
            lastTrackedAnchor = refinedAnchor

            // Prepare next continuous tracking request
            let nextTrack = VNTrackObjectRequest(detectedObjectObservation: observation)
            nextTrack.trackingLevel = .accurate
            self.trackRequest = nextTrack

            DispatchQueue.main.async {
                self.activeTrackedSubject = trackedTarget
                self.isReidentifying = false
            }

        } catch {
            handleTrackingLoss(pixelBuffer: pixelBuffer, timestamp: timestamp, predictedRay: predictedRay)
        }
    }

    // MARK: - 2-Tier Re-Identification
    private func handleTrackingLoss(pixelBuffer: CVPixelBuffer, timestamp: Double, predictedRay: S2Ray?) {
        consecutiveLostFrames += 1

        DispatchQueue.main.async {
            self.isReidentifying = true
        }

        // Tier 1: Fast Saliency Scan (~7Hz, every ~140ms)
        if timestamp - lastSaliencyTime >= 0.14 {
            lastSaliencyTime = timestamp
            if performTier1SaliencyReID(pixelBuffer: pixelBuffer, predictedRay: predictedRay) {
                return
            }
        }

        // Tier 2: Neural FeaturePrint Grid Search if lost for > 3 frames
        if consecutiveLostFrames >= 3 {
            performTier2FeaturePrintGridSearch(pixelBuffer: pixelBuffer)
        }
    }

    private func performTier1SaliencyReID(pixelBuffer: CVPixelBuffer, predictedRay: S2Ray?) -> Bool {
        let saliencyRequest = VNGenerateAttentionBasedSaliencyImageRequest()
        let handler = VNImageRequestHandler(cvPixelBuffer: pixelBuffer, orientation: .up)

        do {
            try handler.perform([saliencyRequest])
            guard let saliencyObservation = saliencyRequest.results?.first as? VNSaliencyImageObservation,
                  let salientObjects = saliencyObservation.salientObjects else {
                return false
            }

            for salient in salientObjects {
                let uiBox = visionRectToUIRect(salient.boundingBox)
                let anchor = CGPoint(x: uiBox.midX, y: uiBox.midY)

                // If close to last known box or predicted bearing ray
                if let lastBox = lastTrackedBox {
                    let dist = hypot(uiBox.midX - lastBox.midX, uiBox.midY - lastBox.midY)
                    if dist < 0.25 {
                        reinitializeTracking(pixelBuffer: pixelBuffer, boundingBox: uiBox, anchor: anchor)
                        return true
                    }
                }
            }
        } catch {
            print("[VisionFramingEngine] Saliency Re-ID error: \(error)")
        }
        return false
    }

    private func performTier2FeaturePrintGridSearch(pixelBuffer: CVPixelBuffer) {
        guard let referencePrint = self.referenceFeaturePrint else { return }

        // 25-cell grid search: 5 columns x 5 rows
        let cols = 5
        let rows = 5
        let cellWidth: CGFloat = 1.0 / CGFloat(cols)
        let cellHeight: CGFloat = 1.0 / CGFloat(rows)

        var bestMatchBox: CGRect?
        var minDistance: Float = Float.greatestFiniteMagnitude

        for r in 0..<rows {
            for c in 0..<cols {
                let cropX = CGFloat(c) * cellWidth
                let cropY = CGFloat(r) * cellHeight
                let testRect = CGRect(x: cropX, y: cropY, width: cellWidth, height: cellHeight)

                let featureRequest = VNGenerateImageFeaturePrintRequest()
                featureRequest.imageCropAndScaleOption = .scaleFit
                featureRequest.regionOfInterest = testRect

                let handler = VNImageRequestHandler(cvPixelBuffer: pixelBuffer, orientation: .up)
                do {
                    try handler.perform([featureRequest])
                    if let result = featureRequest.results?.first as? VNFeaturePrintObservation {
                        var distance: Float = 0
                        try result.computeDistance(&distance, to: referencePrint)

                        if distance < minDistance {
                            minDistance = distance
                            bestMatchBox = testRect
                        }
                    }
                } catch {
                    continue
                }
            }
        }

        // Distance threshold: Re-lock when feature distance is sufficiently small
        if let bestBox = bestMatchBox, minDistance <= 0.40 {
            let uiBox = visionRectToUIRect(bestBox)
            let anchor = CGPoint(x: uiBox.midX, y: uiBox.midY)
            reinitializeTracking(pixelBuffer: pixelBuffer, boundingBox: uiBox, anchor: anchor)
        }
    }

    private func reinitializeTracking(pixelBuffer: CVPixelBuffer, boundingBox: CGRect, anchor: CGPoint) {
        let visionBox = uiRectToVisionRect(boundingBox)
        let obs = VNDetectedObjectObservation(boundingBox: visionBox)
        let request = VNTrackObjectRequest(detectedObjectObservation: obs)
        request.trackingLevel = .accurate

        self.trackRequest = request
        self.lastTrackedBox = boundingBox
        self.lastTrackedAnchor = anchor
        self.consecutiveLostFrames = 0

        let target = TrackedTargetObservation(
            boundingBox: boundingBox,
            anchorPoint: anchor,
            type: .foregroundObject,
            confidence: 0.85,
            gazeVector: CGVector(dx: 0, dy: 0)
        )

        DispatchQueue.main.async {
            self.activeTrackedSubject = target
            self.isReidentifying = false
        }
    }

    // MARK: - Detect Subjects & Eye Anchors
    private func detectSubjects(pixelBuffer: CVPixelBuffer) {
        let faceRequest = VNDetectFaceLandmarksRequest()
        let bodyPoseRequest = VNDetectHumanBodyPoseRequest()

        let handler = VNImageRequestHandler(cvPixelBuffer: pixelBuffer, orientation: .up)

        do {
            try handler.perform([faceRequest, bodyPoseRequest])

            var subjects: [TrackedTargetObservation] = []

            // Process detected faces
            if let faces = faceRequest.results as? [VNFaceObservation] {
                for face in faces {
                    let uiBox = visionRectToUIRect(face.boundingBox)
                    var eyeAnchor = CGPoint(x: uiBox.midX, y: uiBox.midY)
                    var gazeVec = CGVector(dx: 0, dy: 0)

                    if let landmarks = face.landmarks {
                        // Extract left and right pupil / eyes
                        var eyePoints: [CGPoint] = []
                        if let leftEye = landmarks.leftEye {
                            eyePoints.append(contentsOf: leftEye.normalizedPoints.map {
                                CGPoint(
                                    x: uiBox.origin.x + $0.x * uiBox.width,
                                    y: uiBox.origin.y + (1.0 - $0.y) * uiBox.height
                                )
                            })
                        }
                        if let rightEye = landmarks.rightEye {
                            eyePoints.append(contentsOf: rightEye.normalizedPoints.map {
                                CGPoint(
                                    x: uiBox.origin.x + $0.x * uiBox.width,
                                    y: uiBox.origin.y + (1.0 - $0.y) * uiBox.height
                                )
                            })
                        }

                        if !eyePoints.isEmpty {
                            let avgX = eyePoints.reduce(0) { $0 + $1.x } / CGFloat(eyePoints.count)
                            let avgY = eyePoints.reduce(0) { $0 + $1.y } / CGFloat(eyePoints.count)
                            eyeAnchor = CGPoint(x: avgX, y: avgY)
                        }

                        // Approximate face gaze from yaw
                        if let yaw = face.yaw {
                            let yawVal = yaw.doubleValue
                            gazeVec = CGVector(dx: sin(yawVal), dy: 0)
                        }
                    }

                    subjects.append(TrackedTargetObservation(
                        boundingBox: uiBox,
                        anchorPoint: eyeAnchor,
                        type: .face,
                        confidence: face.confidence,
                        gazeVector: gazeVec
                    ))
                }
            }

            // Process detected human bodies
            if let bodies = bodyPoseRequest.results as? [VNHumanBodyPoseObservation] {
                for body in bodies {
                    if let recognizedPoints = try? body.recognizedPoints(.all) {
                        let validPoints = recognizedPoints.values.filter { $0.confidence > 0.3 }
                        if !validPoints.isEmpty {
                            let minX = validPoints.map { $0.location.x }.min() ?? 0
                            let maxX = validPoints.map { $0.location.x }.max() ?? 1
                            let minY = validPoints.map { $0.location.y }.min() ?? 0
                            let maxY = validPoints.map { $0.location.y }.max() ?? 1

                            let bodyVisionRect = CGRect(x: minX, y: minY, width: maxX - minX, height: maxY - minY)
                            let uiBox = visionRectToUIRect(bodyVisionRect)
                            let anchor = CGPoint(x: uiBox.midX, y: uiBox.minY + uiBox.height * 0.2) // Upper chest / head anchor

                            // Don't add duplicate if face already covers this
                            let overlapsWithFace = subjects.contains { $0.boundingBox.intersects(uiBox) }
                            if !overlapsWithFace {
                                subjects.append(TrackedTargetObservation(
                                    boundingBox: uiBox,
                                    anchorPoint: anchor,
                                    type: .human,
                                    confidence: 0.8,
                                    gazeVector: CGVector(dx: 0, dy: 0)
                                ))
                            }
                        }
                    }
                }
            }

            DispatchQueue.main.async {
                self.currentDetectedSubjects = subjects
            }
        } catch {
            print("[VisionFramingEngine] Detect subjects error: \(error)")
        }
    }

    private func detectEyeAnchorOrCenter(pixelBuffer: CVPixelBuffer, insideBox visionBox: CGRect) -> (anchor: CGPoint, gaze: CGVector) {
        let faceRequest = VNDetectFaceLandmarksRequest()
        faceRequest.regionOfInterest = visionBox

        let handler = VNImageRequestHandler(cvPixelBuffer: pixelBuffer, orientation: .up)
        let uiBox = visionRectToUIRect(visionBox)

        do {
            try handler.perform([faceRequest])
            if let face = faceRequest.results?.first as? VNFaceObservation, let landmarks = face.landmarks {
                var eyePoints: [CGPoint] = []
                if let leftEye = landmarks.leftEye {
                    eyePoints.append(contentsOf: leftEye.normalizedPoints.map {
                        CGPoint(
                            x: uiBox.origin.x + $0.x * uiBox.width,
                            y: uiBox.origin.y + (1.0 - $0.y) * uiBox.height
                        )
                    })
                }
                if let rightEye = landmarks.rightEye {
                    eyePoints.append(contentsOf: rightEye.normalizedPoints.map {
                        CGPoint(
                            x: uiBox.origin.x + $0.x * uiBox.width,
                            y: uiBox.origin.y + (1.0 - $0.y) * uiBox.height
                        )
                    })
                }

                if !eyePoints.isEmpty {
                    let avgX = eyePoints.reduce(0) { $0 + $1.x } / CGFloat(eyePoints.count)
                    let avgY = eyePoints.reduce(0) { $0 + $1.y } / CGFloat(eyePoints.count)
                    let gazeX = face.yaw != nil ? sin(face.yaw!.doubleValue) : 0
                    return (CGPoint(x: avgX, y: avgY), CGVector(dx: gazeX, dy: 0))
                }
            }
        } catch {}

        return (CGPoint(x: uiBox.midX, y: uiBox.midY), CGVector(dx: 0, dy: 0))
    }

    private func extractFeaturePrint(from pixelBuffer: CVPixelBuffer, cropRect: CGRect) {
        let request = VNGenerateImageFeaturePrintRequest()
        request.imageCropAndScaleOption = .scaleFit
        request.regionOfInterest = cropRect

        let handler = VNImageRequestHandler(cvPixelBuffer: pixelBuffer, orientation: .up)
        do {
            try handler.perform([request])
            self.referenceFeaturePrint = request.results?.first as? VNFeaturePrintObservation
        } catch {
            print("[VisionFramingEngine] Feature print extraction failed: \(error)")
        }
    }

    // MARK: - Coordinate Transformations
    public func visionRectToUIRect(_ vRect: CGRect) -> CGRect {
        // Vision: (0,0) bottom-left -> UI: (0,0) top-left
        return CGRect(
            x: vRect.origin.x,
            y: 1.0 - vRect.origin.y - vRect.height,
            width: vRect.width,
            height: vRect.height
        )
    }

    public func uiRectToVisionRect(_ uRect: CGRect) -> CGRect {
        // UI: (0,0) top-left -> Vision: (0,0) bottom-left
        return CGRect(
            x: uRect.origin.x,
            y: 1.0 - uRect.origin.y - uRect.height,
            width: uRect.width,
            height: uRect.height
        )
    }
}
