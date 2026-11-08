import SwiftUI

public struct ARFramingOverlayView: View {
    @ObservedObject var viewModel: CameraViewModel

    @State private var radarSweepProgress: CGFloat = 0.0
    @State private var sunDragOffset: CGFloat = 0.0
    @State private var isDraggingSun: Bool = false

    public init(viewModel: CameraViewModel) {
        self.viewModel = viewModel
    }

    public var body: some View {
        GeometryReader { geometry in
            let size = geometry.size

            ZStack {
                // 1. Dynamic Composition Grid Lines
                CompositionGridLinesView(rule: viewModel.activeCompositionRule)

                // 2. Scan Radar Wave Sweep View (when analyzing)
                if viewModel.sessionState == .analyzing {
                    ScanRadarWaveSweepView(progress: radarSweepProgress)
                        .onAppear {
                            withAnimation(.linear(duration: 2.5).repeatForever(autoreverses: false)) {
                                radarSweepProgress = 1.0
                            }
                        }
                }

                // 3. Multi-entity Bounding Boxes (Tap-to-Track)
                ForEach(viewModel.visionEngine.currentDetectedSubjects.indices, id: \.self) { idx in
                    let subject = viewModel.visionEngine.currentDetectedSubjects[idx]
                    SubjectBoundingBoxView(subject: subject, containerSize: size) {
                        viewModel.userTappedSubject(subject)
                    }
                }

                // 4. Virtual Horizon Leveler
                VirtualHorizonLevelerView(
                    rollDegrees: viewModel.motionService.currentRollDegrees,
                    isLevel: viewModel.motionService.isLevel,
                    containerSize: size
                )

                // 5. Center Alignment Ring (0.5, 0.5)
                CenterAlignmentRingView(isPerfect: viewModel.sessionState == .alignmentPerfect)
                    .position(x: size.width * 0.5, y: size.height * 0.5)

                // 6. 3D Spatial Target Gold Reticle (Aim Point)
                if case .targetPlaced = viewModel.sessionState {
                    targetReticle(containerSize: size)
                } else if viewModel.sessionState == .alignmentPerfect {
                    targetReticle(containerSize: size)
                }

                // 7. Sun Exposure Slider (Interactive EV bias adjustment)
                if isDraggingSun || viewModel.sessionState == .alignmentPerfect {
                    sunExposureControl(containerSize: size)
                }
            }
            .contentShape(Rectangle())
        }
        .allowsHitTesting(true)
    }

    // MARK: - 3D Spatial Target Reticle View with Edge Docking
    @ViewBuilder
    private func targetReticle(containerSize: CGSize) -> some View {
        let posX = viewModel.currentTargetScreenPoint.x * containerSize.width
        let posY = viewModel.currentTargetScreenPoint.y * containerSize.height

        ZStack {
            if viewModel.isTargetOffscreen {
                // Offscreen Edge Docking with Directional Arrow
                VStack(spacing: 2) {
                    Image(systemName: "chevron.up.circle.fill")
                        .font(.system(size: 20, weight: .bold))
                        .foregroundColor(Color(red: 1.0, green: 0.84, blue: 0.0)) // #FFD700
                        .rotationEffect(.radians(viewModel.offscreenDockAngleRad - (.pi / 2)))

                    Text("PAN")
                        .font(.system(size: 9, weight: .black, design: .monospaced))
                        .foregroundColor(.white)
                        .padding(.horizontal, 4)
                        .padding(.vertical, 1)
                        .background(Color.black.opacity(0.75))
                        .cornerRadius(3)
                }
                .shadow(color: Color.black.opacity(0.8), radius: 4)
            } else {
                // 3D Gold Aim Reticle (36pt)
                ZStack {
                    // Outer golden ring
                    Circle()
                        .stroke(
                            viewModel.sessionState == .alignmentPerfect
                                ? Color.green
                                : Color(red: 1.0, green: 0.84, blue: 0.0),
                            lineWidth: 2.2
                        )
                        .frame(width: 36, height: 36)
                        .shadow(color: Color.yellow.opacity(0.6), radius: 6)

                    // 4 Micro Crosshair Ticks
                    ForEach([0, 90, 180, 270], id: \.self) { deg in
                        Rectangle()
                            .fill(Color(red: 1.0, green: 0.84, blue: 0.0))
                            .frame(width: 1.5, height: 5)
                            .offset(y: -18)
                            .rotationEffect(.degrees(Double(deg)))
                    }

                    // Inner Dot
                    Circle()
                        .fill(viewModel.sessionState == .alignmentPerfect ? Color.green : Color.yellow)
                        .frame(width: 4, height: 4)
                }
            }
        }
        .position(x: posX, y: posY)
        .animation(.interactiveSpring(response: 0.25, dampingFraction: 0.85), value: viewModel.currentTargetScreenPoint)
    }

    // MARK: - Sun Exposure Slider
    @ViewBuilder
    private func sunExposureControl(containerSize: CGSize) -> some View {
        let posX = min(containerSize.width - 40, viewModel.currentTargetScreenPoint.x * containerSize.width + 48)
        let posY = viewModel.currentTargetScreenPoint.y * containerSize.height

        VStack(spacing: 8) {
            Image(systemName: "sun.max.fill")
                .foregroundColor(.yellow)
                .font(.system(size: 16))

            // Exposure readout
            Text(String(format: "%+.1f", viewModel.exposureBias))
                .font(.system(size: 10, weight: .bold, design: .monospaced))
                .foregroundColor(.white)
                .padding(.horizontal, 4)
                .background(Color.black.opacity(0.6))
                .cornerRadius(3)
        }
        .position(x: posX, y: posY + sunDragOffset)
        .gesture(
            DragGesture()
                .onChanged { val in
                    isDraggingSun = true
                    sunDragOffset = val.translation.height
                    // Drag up increases EV, drag down decreases EV (range -2.0 to +2.0)
                    let deltaEV = Float(-val.translation.height / 50.0)
                    let newBias = max(-2.0, min(2.0, deltaEV))
                    viewModel.setExposureBias(newBias)
                }
                .onEnded { _ in
                    isDraggingSun = false
                    sunDragOffset = 0
                }
        )
    }
}

// MARK: - Composition Grid Lines
public struct CompositionGridLinesView: View {
    let rule: CompositionRuleType

    public var body: some View {
        GeometryReader { geo in
            let w = geo.size.width
            let h = geo.size.height

            Path { path in
                switch rule {
                case .ruleOfThirds:
                    // Vertical lines
                    path.move(to: CGPoint(x: w / 3, y: 0))
                    path.addLine(to: CGPoint(x: w / 3, y: h))
                    path.move(to: CGPoint(x: 2 * w / 3, y: 0))
                    path.addLine(to: CGPoint(x: 2 * w / 3, y: h))
                    // Horizontal lines
                    path.move(to: CGPoint(x: 0, y: h / 3))
                    path.addLine(to: CGPoint(x: w, y: h / 3))
                    path.move(to: CGPoint(x: 0, y: 2 * h / 3))
                    path.addLine(to: CGPoint(x: w, y: 2 * h / 3))

                case .goldenRatio:
                    let phi1: CGFloat = 0.382
                    let phi2: CGFloat = 0.618
                    // Vertical phi lines
                    path.move(to: CGPoint(x: w * phi1, y: 0))
                    path.addLine(to: CGPoint(x: w * phi1, y: h))
                    path.move(to: CGPoint(x: w * phi2, y: 0))
                    path.addLine(to: CGPoint(x: w * phi2, y: h))
                    // Horizontal phi lines
                    path.move(to: CGPoint(x: 0, y: h * phi1))
                    path.addLine(to: CGPoint(x: w, y: h * phi1))
                    path.move(to: CGPoint(x: 0, y: h * phi2))
                    path.addLine(to: CGPoint(x: w, y: h * phi2))

                case .centerSymmetry:
                    // Center crosshair
                    path.move(to: CGPoint(x: w * 0.5, y: 0))
                    path.addLine(to: CGPoint(x: w * 0.5, y: h))
                    path.move(to: CGPoint(x: 0, y: h * 0.5))
                    path.addLine(to: CGPoint(x: w, y: h * 0.5))

                case .leadingLines:
                    // Diagonal lines
                    path.move(to: CGPoint(x: 0, y: h))
                    path.addLine(to: CGPoint(x: w, y: 0))
                    path.move(to: CGPoint(x: 0, y: 0))
                    path.addLine(to: CGPoint(x: w, y: h))
                }
            }
            .stroke(Color.white.opacity(0.18), lineWidth: 0.8)
        }
    }
}

// MARK: - Scan Radar Sweep
public struct ScanRadarWaveSweepView: View {
    var progress: CGFloat

    public var body: some View {
        GeometryReader { geo in
            let y = geo.size.height * progress

            Rectangle()
                .fill(
                    LinearGradient(
                        gradient: Gradient(colors: [
                            Color.cyan.opacity(0.0),
                            Color.cyan.opacity(0.25),
                            Color.cyan.opacity(0.0)
                        ]),
                        startPoint: .top,
                        endPoint: .bottom
                    )
                )
                .frame(height: 60)
                .position(x: geo.size.width * 0.5, y: y)
        }
        .allowsHitTesting(false)
    }
}

// MARK: - Subject Bounding Box View
public struct SubjectBoundingBoxView: View {
    let subject: TrackedTargetObservation
    let containerSize: CGSize
    let onTap: () -> Void

    public var body: some View {
        let box = subject.boundingBox
        let rect = CGRect(
            x: box.origin.x * containerSize.width,
            y: box.origin.y * containerSize.height,
            width: box.size.width * containerSize.width,
            height: box.size.height * containerSize.height
        )

        ZStack(alignment: .topLeading) {
            // Rounded Bounding Box with Corner Accents
            RoundedRectangle(cornerRadius: 6)
                .stroke(Color.white.opacity(0.55), style: StrokeStyle(lineWidth: 1.2, dash: [4, 4]))
                .frame(width: rect.width, height: rect.height)

            // Subject Tag Label
            HStack(spacing: 3) {
                Image(systemName: subject.type == .face ? "face.smiling" : "person.fill")
                    .font(.system(size: 8))
                Text(subject.type.rawValue)
                    .font(.system(size: 9, weight: .bold))
            }
            .foregroundColor(.black)
            .padding(.horizontal, 4)
            .padding(.vertical, 2)
            .background(Color.yellow.opacity(0.9))
            .cornerRadius(3)
            .offset(y: -14)
        }
        .position(x: rect.midX, y: rect.midY)
        .onTapGesture {
            onTap()
        }
    }
}

// MARK: - Virtual Horizon Leveler
public struct VirtualHorizonLevelerView: View {
    let rollDegrees: Double
    let isLevel: Bool
    let containerSize: CGSize

    public var body: some View {
        let barColor = isLevel ? Color.yellow : Color.white.opacity(0.4)
        let barWidth: CGFloat = 60
        let centerY = containerSize.height * 0.5

        HStack {
            // Left horizon indicator
            Rectangle()
                .fill(barColor)
                .frame(width: barWidth, height: 1.5)

            Spacer()

            // Right horizon indicator
            Rectangle()
                .fill(barColor)
                .frame(width: barWidth, height: 1.5)
        }
        .padding(.horizontal, containerSize.width * 0.18)
        .position(x: containerSize.width * 0.5, y: centerY)
        .rotationEffect(.degrees(-rollDegrees))
        .animation(.easeOut(duration: 0.1), value: rollDegrees)
    }
}

// MARK: - Center Alignment Ring
public struct CenterAlignmentRingView: View {
    let isPerfect: Bool

    public var body: some View {
        ZStack {
            // Outer Ring
            Circle()
                .stroke(isPerfect ? Color.green : Color.white.opacity(0.7), lineWidth: 1.8)
                .frame(width: 44, height: 44)
                .scaleEffect(isPerfect ? 1.15 : 1.0)
                .shadow(color: isPerfect ? Color.green.opacity(0.8) : Color.clear, radius: 8)

            // Center cross dot
            Circle()
                .fill(isPerfect ? Color.green : Color.white.opacity(0.8))
                .frame(width: 4, height: 4)
        }
        .animation(.spring(response: 0.3, dampingFraction: 0.7), value: isPerfect)
    }
}
