import SwiftUI

public struct ContentView: View {
    @StateObject private var viewModel = CameraViewModel()
    @State private var showingPhotoSheet: Bool = false

    public init() {}

    public var body: some View {
        ZStack {
            Color.black.ignoresSafeArea()

            // 1. Camera Live Viewfinder
            CameraPreviewView(session: viewModel.cameraService.captureSession)
                .ignoresSafeArea()

            // 2. AR 3D Spatial Framing Overlay
            ARFramingOverlayView(viewModel: viewModel)
                .ignoresSafeArea()

            // 3. UI Controls HUD
            VStack {
                topControlBar
                Spacer()
                bottomControlBar
            }
            .padding(.horizontal, 16)
            .padding(.top, 8)
            .padding(.bottom, 20)
        }
        .onAppear {
            viewModel.startCamera()
        }
        .onDisappear {
            viewModel.stopCamera()
        }
        .sheet(isPresented: $showingPhotoSheet) {
            if let photo = viewModel.lastCapturedPhoto {
                PhotoPreviewSheet(image: photo)
            }
        }
    }

    // MARK: - Top Control Bar
    private var topControlBar: some View {
        HStack(spacing: 12) {
            // Composition Rule Menu
            Menu {
                ForEach(CompositionRuleType.allCases, id: \.self) { rule in
                    Button {
                        viewModel.activeCompositionRule = rule
                    } label: {
                        HStack {
                            Text(rule.rawValue)
                            if viewModel.activeCompositionRule == rule {
                                Image(systemName: "checkmark")
                            }
                        }
                    }
                }
            } label: {
                HStack(spacing: 4) {
                    Image(systemName: "grid")
                        .font(.system(size: 13, weight: .semibold))
                    Text(viewModel.activeCompositionRule.rawValue)
                        .font(.system(size: 12, weight: .bold))
                }
                .foregroundColor(.white)
                .padding(.horizontal, 10)
                .padding(.vertical, 6)
                .background(Color.black.opacity(0.55))
                .clipShape(Capsule())
            }

            Spacer()

            // Auto-Capture Toggle
            Button {
                viewModel.autoCaptureEnabled.toggle()
            } label: {
                HStack(spacing: 4) {
                    Image(systemName: viewModel.autoCaptureEnabled ? "bolt.badge.automatic.fill" : "bolt.slash.fill")
                        .font(.system(size: 13))
                    Text(viewModel.autoCaptureEnabled ? "AUTO" : "MANUAL")
                        .font(.system(size: 11, weight: .heavy, design: .monospaced))
                }
                .foregroundColor(viewModel.autoCaptureEnabled ? .yellow : .white.opacity(0.7))
                .padding(.horizontal, 10)
                .padding(.vertical, 6)
                .background(Color.black.opacity(0.55))
                .clipShape(Capsule())
            }

            // Target Reset Button
            if case .targetPlaced = viewModel.sessionState {
                Button {
                    viewModel.resetTarget()
                } label: {
                    Image(systemName: "arrow.counterclockwise.circle.fill")
                        .font(.system(size: 22))
                        .foregroundColor(.white)
                }
            }
        }
    }

    // MARK: - Bottom Control Bar
    private var bottomControlBar: some View {
        VStack(spacing: 16) {
            // Lens Picker (0.5x, 1x, 2x, 5x)
            HStack(spacing: 14) {
                ForEach([0.5, 1.0, 2.0, 5.0], id: \.self) { zoom in
                    Button {
                        viewModel.setZoom(CGFloat(zoom))
                    } label: {
                        Text(zoom == 0.5 ? ".5" : String(format: "%.0f", zoom))
                            + Text("x").font(.system(size: 10, weight: .bold))
                    }
                    .font(.system(size: 13, weight: .bold, design: .rounded))
                    .foregroundColor(viewModel.currentZoom == CGFloat(zoom) ? .yellow : .white)
                    .frame(width: 36, height: 36)
                    .background(
                        Circle()
                            .fill(viewModel.currentZoom == CGFloat(zoom) ? Color.black.opacity(0.7) : Color.black.opacity(0.4))
                    )
                    .overlay(
                        Circle()
                            .stroke(viewModel.currentZoom == CGFloat(zoom) ? Color.yellow : Color.clear, lineWidth: 1.5)
                    )
                }
            }

            // Shutter Button & Gallery Thumbnail
            HStack {
                // Last Captured Thumbnail
                if let photo = viewModel.lastCapturedPhoto {
                    Button {
                        showingPhotoSheet = true
                    } label: {
                        Image(uiImage: photo)
                            .resizable()
                            .scaledToFill()
                            .frame(width: 48, height: 48)
                            .clipShape(RoundedRectangle(cornerRadius: 8))
                            .overlay(RoundedRectangle(cornerRadius: 8).stroke(Color.white, lineWidth: 1.5))
                    }
                } else {
                    Circle()
                        .fill(Color.white.opacity(0.15))
                        .frame(width: 48, height: 48)
                }

                Spacer()

                // Main Shutter Button
                Button {
                    viewModel.takePhoto()
                } label: {
                    ZStack {
                        Circle()
                            .stroke(Color.white, lineWidth: 4)
                            .frame(width: 74, height: 74)

                        Circle()
                            .fill(viewModel.sessionState == .alignmentPerfect ? Color.yellow : Color.white)
                            .frame(width: 60, height: 60)
                            .scaleEffect(viewModel.sessionState == .capturing ? 0.85 : 1.0)
                    }
                }
                .disabled(viewModel.sessionState == .capturing)

                Spacer()

                // Status Badge
                VStack(alignment: .trailing, spacing: 2) {
                    Text(statusLabel)
                        .font(.system(size: 10, weight: .bold, design: .monospaced))
                        .foregroundColor(statusColor)
                }
                .frame(width: 48)
            }
            .padding(.horizontal, 20)
        }
    }

    private var statusLabel: String {
        switch viewModel.sessionState {
        case .idle: return "IDLE"
        case .analyzing: return "SCANNING"
        case .targetPlaced: return "LOCKED"
        case .alignmentPerfect: return "PERFECT"
        case .capturing: return "EXPOSING"
        case .done: return "SAVED"
        }
    }

    private var statusColor: Color {
        switch viewModel.sessionState {
        case .idle, .analyzing: return .white.opacity(0.8)
        case .targetPlaced: return .yellow
        case .alignmentPerfect: return .green
        case .capturing: return .orange
        case .done: return .cyan
        }
    }
}

// MARK: - Captured Photo Preview Sheet
public struct PhotoPreviewSheet: View {
    let image: UIImage
    @Environment(\.dismiss) private var dismiss

    public var body: some View {
        NavigationView {
            ZStack {
                Color.black.ignoresSafeArea()

                Image(uiImage: image)
                    .resizable()
                    .scaledToFit()
                    .ignoresSafeArea()
            }
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .navigationBarTrailing) {
                    Button("Done") {
                        dismiss()
                    }
                    .foregroundColor(.yellow)
                }
            }
        }
    }
}
