import SwiftUI
import AVFoundation

/// UIView subclass holding the AVCaptureVideoPreviewLayer
public final class VideoPreviewUIView: UIView {
    public override class var layerClass: AnyClass {
        return AVCaptureVideoPreviewLayer.self
    }

    public var videoPreviewLayer: AVCaptureVideoPreviewLayer {
        return layer as! AVCaptureVideoPreviewLayer
    }
}

/// SwiftUI wrapper for AVCaptureVideoPreviewLayer with resizeAspectFill
public struct CameraPreviewView: UIViewRepresentable {
    private let captureSession: AVCaptureSession

    public init(session: AVCaptureSession) {
        self.captureSession = session
    }

    public func makeUIView(context: Context) -> VideoPreviewUIView {
        let view = VideoPreviewUIView()
        view.backgroundColor = .black
        view.videoPreviewLayer.session = captureSession
        view.videoPreviewLayer.videoGravity = .resizeAspectFill
        view.videoPreviewLayer.connection?.videoOrientation = .portrait
        return view
    }

    public func updateUIView(_ uiView: VideoPreviewUIView, context: Context) {
        if uiView.videoPreviewLayer.session != captureSession {
            uiView.videoPreviewLayer.session = captureSession
        }
    }
}
