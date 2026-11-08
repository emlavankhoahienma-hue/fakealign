import Foundation
import AVFoundation
import CoreMedia
import UIKit
import Combine

/// Protocol for listening to real-time camera video frames
public protocol CameraServiceDelegate: AnyObject {
    func cameraService(_ service: CameraService, didOutput sampleBuffer: CMSampleBuffer)
}

/// CameraService controls AVFoundation camera pipeline at 60 FPS
public final class CameraService: NSObject {
    public static let shared = CameraService()

    // MARK: - Published State
    public let sampleBufferPublisher = PassthroughSubject<CMSampleBuffer, Never>()
    public let photoCapturePublisher = PassthroughSubject<UIImage, Never>()
    public private(set) var currentZoomFactor: CGFloat = 1.0
    public private(set) var isSessionRunning: Bool = false

    // MARK: - Delegates
    public weak var delegate: CameraServiceDelegate?

    // MARK: - AVFoundation Internals
    public let captureSession = AVCaptureSession()
    private var videoDeviceInput: AVCaptureDeviceInput?
    private let videoDataOutput = AVCaptureVideoDataOutput()
    private let photoOutput = AVCapturePhotoOutput()

    private let sessionQueue = DispatchQueue(label: "com.fakealign.camera.sessionQueue", qos: .userInitiated)
    private let videoDataQueue = DispatchQueue(label: "com.fakealign.camera.videoDataQueue", qos: .userInteractive)

    private var activeDevice: AVCaptureDevice?
    private var photoCompletionHandler: ((Result<UIImage, Error>) -> Void)?

    private override init() {
        super.init()
    }

    // MARK: - Setup
    public func startSession() {
        sessionQueue.async { [weak self] in
            guard let self = self else { return }
            if self.isSessionRunning { return }

            if self.captureSession.inputs.isEmpty {
                self.configureSession()
            }

            self.captureSession.startRunning()
            self.isSessionRunning = self.captureSession.isRunning
        }
    }

    public func stopSession() {
        sessionQueue.async { [weak self] in
            guard let self = self, self.isSessionRunning else { return }
            self.captureSession.stopRunning()
            self.isSessionRunning = false
        }
    }

    private func configureSession() {
        captureSession.beginConfiguration()
        if captureSession.canSetSessionPreset(.hd1920x1080) {
            captureSession.sessionPreset = .hd1920x1080
        } else {
            captureSession.sessionPreset = .high
        }

        // Discover optimal back camera device
        let deviceDiscovery = AVCaptureDevice.DiscoverySession(
            deviceTypes: [
                .builtInTripleCamera,
                .builtInDualWideCamera,
                .builtInDualCamera,
                .builtInWideAngleCamera
            ],
            mediaType: .video,
            position: .back
        )

        guard let device = deviceDiscovery.devices.first ?? AVCaptureDevice.default(for: .video) else {
            print("[CameraService] No compatible camera device found.")
            captureSession.commitConfiguration()
            return
        }

        self.activeDevice = device

        do {
            let input = try AVCaptureDeviceInput(device: device)
            if captureSession.canAddInput(input) {
                captureSession.addInput(input)
                self.videoDeviceInput = input
            }

            // Configure 60 FPS if supported
            try device.lockForConfiguration()
            let targetFrameDuration = CMTime(value: 1, timescale: 60)
            for format in device.formats {
                let ranges = format.videoSupportedFrameRateRanges
                for range in ranges where range.maxFrameRate >= 60.0 {
                    if format.formatDescription.dimensions.width >= 1920 {
                        device.activeFormat = format
                        device.activeVideoMinFrameDuration = targetFrameDuration
                        device.activeVideoMaxFrameDuration = targetFrameDuration
                        break
                    }
                }
            }
            device.unlockForConfiguration()
        } catch {
            print("[CameraService] Failed to configure camera device input: \(error)")
        }

        // Configure Video Output
        videoDataOutput.videoSettings = [
            (kCVPixelBufferPixelFormatTypeKey as String): Int(kCVPixelFormatType_420YpCbCr8BiPlanarFullRange)
        ]
        videoDataOutput.alwaysDiscardsLateVideoFrames = true
        videoDataOutput.setSampleBufferDelegate(self, queue: videoDataQueue)

        if captureSession.canAddOutput(videoDataOutput) {
            captureSession.addOutput(videoDataOutput)
        }

        // Enable Camera Intrinsics Delivery on video connection
        if let videoConnection = videoDataOutput.connection(with: .video) {
            if videoConnection.isVideoOrientationSupported {
                videoConnection.videoOrientation = .portrait
            }
            if videoConnection.isCameraIntrinsicMatrixDeliverySupported {
                videoConnection.isCameraIntrinsicMatrixDeliveryEnabled = true
            }
        }

        // Configure Photo Output
        photoOutput.maxPhotoQualityPrioritization = .quality
        if captureSession.canAddOutput(photoOutput) {
            captureSession.addOutput(photoOutput)
        }

        captureSession.commitConfiguration()
    }

    // MARK: - Smart Zoom Control
    public func rampZoom(to displayZoom: CGFloat, rate: Float = 4.0) {
        sessionQueue.async { [weak self] in
            guard let self = self, let device = self.activeDevice else { return }
            do {
                try device.lockForConfiguration()
                let minZoom = device.minAvailableVideoZoomFactor
                let maxZoom = min(device.maxAvailableVideoZoomFactor, 10.0)
                let target = max(minZoom, min(maxZoom, displayZoom))

                device.ramp(toVideoZoomFactor: target, withRate: rate)
                self.currentZoomFactor = target
                device.unlockForConfiguration()
            } catch {
                print("[CameraService] Failed to ramp zoom: \(error)")
            }
        }
    }

    public func setLinearZoom(_ factor: CGFloat) {
        sessionQueue.async { [weak self] in
            guard let self = self, let device = self.activeDevice else { return }
            do {
                try device.lockForConfiguration()
                let minZoom = device.minAvailableVideoZoomFactor
                let maxZoom = min(device.maxAvailableVideoZoomFactor, 10.0)
                let target = max(minZoom, min(maxZoom, factor))

                device.videoZoomFactor = target
                self.currentZoomFactor = target
                device.unlockForConfiguration()
            } catch {
                print("[CameraService] Failed to set linear zoom: \(error)")
            }
        }
    }

    // MARK: - Smart Focus & Exposure Control
    public func setFocusAndExposure(at normalizedPoint: CGPoint) {
        sessionQueue.async { [weak self] in
            guard let self = self, let device = self.activeDevice else { return }
            do {
                try device.lockForConfiguration()
                let clampedPoint = CGPoint(
                    x: max(0.0, min(1.0, normalizedPoint.x)),
                    y: max(0.0, min(1.0, normalizedPoint.y))
                )

                if device.isFocusPointOfInterestSupported && device.isFocusModeSupported(.continuousAutoFocus) {
                    device.focusPointOfInterest = clampedPoint
                    device.focusMode = .continuousAutoFocus
                }

                if device.isExposurePointOfInterestSupported && device.isExposureModeSupported(.continuousAutoExposure) {
                    device.exposurePointOfInterest = clampedPoint
                    device.exposureMode = .continuousAutoExposure
                }

                device.unlockForConfiguration()
            } catch {
                print("[CameraService] Focus/Exposure setting error: \(error)")
            }
        }
    }

    public func setExposureBias(_ bias: Float) {
        sessionQueue.async { [weak self] in
            guard let self = self, let device = self.activeDevice else { return }
            do {
                try device.lockForConfiguration()
                let clampedBias = max(device.minExposureTargetBias, min(device.maxExposureTargetBias, bias))
                device.setExposureTargetBias(clampedBias, completionHandler: nil)
                device.unlockForConfiguration()
            } catch {
                print("[CameraService] Exposure bias setting error: \(error)")
            }
        }
    }

    // MARK: - Photo Capture
    public func capturePhoto(completion: @escaping (Result<UIImage, Error>) -> Void) {
        sessionQueue.async { [weak self] in
            guard let self = self else { return }
            self.photoCompletionHandler = completion

            var settings = AVCapturePhotoSettings()
            if self.photoOutput.availablePhotoCodecTypes.contains(.hevc) {
                settings = AVCapturePhotoSettings(format: [AVVideoCodecKey: AVVideoCodecType.hevc])
            }
            settings.photoQualityPrioritization = .quality

            self.photoOutput.capturePhoto(with: settings, delegate: self)
        }
    }
}

// MARK: - AVCaptureVideoDataOutputSampleBufferDelegate
extension CameraService: AVCaptureVideoDataOutputSampleBufferDelegate {
    public func captureOutput(_ output: AVCaptureOutput,
                              didOutput sampleBuffer: CMSampleBuffer,
                              from connection: AVCaptureConnection) {
        delegate?.cameraService(self, didOutput: sampleBuffer)
        sampleBufferPublisher.send(sampleBuffer)
    }

    public func captureOutput(_ output: AVCaptureOutput,
                              didDrop sampleBuffer: CMSampleBuffer,
                              from connection: AVCaptureConnection) {
        // Drop notification for diagnostic profiling if needed
    }
}

// MARK: - AVCapturePhotoCaptureDelegate
extension CameraService: AVCapturePhotoCaptureDelegate {
    public func photoOutput(_ output: AVCapturePhotoOutput,
                            didFinishProcessingPhoto photo: AVCapturePhoto,
                            error: Error?) {
        if let error = error {
            photoCompletionHandler?(.failure(error))
            photoCompletionHandler = nil
            return
        }

        guard let data = photo.fileDataRepresentation(),
              let image = UIImage(data: data) else {
            let error = NSError(domain: "com.fakealign.camera",
                                code: -1,
                                userInfo: [NSLocalizedDescriptionKey: "Failed to decode captured photo data."])
            photoCompletionHandler?(.failure(error))
            photoCompletionHandler = nil
            return
        }

        photoCompletionHandler?(.success(image))
        photoCompletionHandler = nil
        photoCapturePublisher.send(image)
    }
}
