import Foundation
import CoreGraphics

/// Types of artistic composition rules
public enum CompositionRuleType: String, CaseIterable {
    case ruleOfThirds = "Rule of Thirds"
    case goldenRatio = "Golden Ratio (Phi)"
    case centerSymmetry = "Center Symmetry"
    case leadingLines = "Leading Lines"
}

/// Composition candidate hypothesis with score and calculated target aim point
public struct CompositionHypothesis: Identifiable {
    public let id = UUID()
    public let rule: CompositionRuleType
    public let gridPoint: CGPoint         // Intended resting position of subject in frame (0..1)
    public let aimPoint: CGPoint          // Point user should move camera center to
    public let score: Double              // Composite score from gaze, headroom, and balance
    public let recommendedZoom: CGFloat   // Recommended optical zoom factor
}

/// CompositionCalculator acts as an autonomous master photographer brain
public final class CompositionCalculator {
    public static let shared = CompositionCalculator()

    private init() {}

    // Supported lens optical zoom factors on modern iPhones
    public let availableZoomFactors: [CGFloat] = [0.5, 1.0, 2.0, 3.0, 5.0]

    // MARK: - Evaluate Best Composition
    public func evaluateOptimalComposition(
        subjectBox: CGRect,
        subjectAnchor: CGPoint,
        gazeVector: CGVector,
        subjectType: TrackedSubjectType,
        currentZoom: CGFloat
    ) -> CompositionHypothesis {
        var hypotheses: [CompositionHypothesis] = []

        // 1. Generate Rule of Thirds candidates
        let thirdsPoints = [
            CGPoint(x: 1.0 / 3.0, y: 1.0 / 3.0),
            CGPoint(x: 2.0 / 3.0, y: 1.0 / 3.0),
            CGPoint(x: 1.0 / 3.0, y: 2.0 / 3.0),
            CGPoint(x: 2.0 / 3.0, y: 2.0 / 3.0)
        ]
        for pt in thirdsPoints {
            hypotheses.append(createHypothesis(rule: .ruleOfThirds, gridPoint: pt, subjectBox: subjectBox, anchor: subjectAnchor, gaze: gazeVector, currentZoom: currentZoom))
        }

        // 2. Generate Golden Ratio (Phi Grid: ~0.382 and ~0.618)
        let phiPoints = [
            CGPoint(x: 0.382, y: 0.382),
            CGPoint(x: 0.618, y: 0.382),
            CGPoint(x: 0.382, y: 0.618),
            CGPoint(x: 0.618, y: 0.618)
        ]
        for pt in phiPoints {
            hypotheses.append(createHypothesis(rule: .goldenRatio, gridPoint: pt, subjectBox: subjectBox, anchor: subjectAnchor, gaze: gazeVector, currentZoom: currentZoom))
        }

        // 3. Generate Center Symmetry candidate
        hypotheses.append(createHypothesis(rule: .centerSymmetry, gridPoint: CGPoint(x: 0.5, y: 0.5), subjectBox: subjectBox, anchor: subjectAnchor, gaze: gazeVector, currentZoom: currentZoom))

        // Find candidate with maximum score
        let best = hypotheses.max { $0.score < $1.score } ?? hypotheses[0]
        return best
    }

    // MARK: - Hypothesis Generator & Scorer
    private func createHypothesis(
        rule: CompositionRuleType,
        gridPoint: CGPoint,
        subjectBox: CGRect,
        anchor: CGPoint,
        gaze: CGVector,
        currentZoom: CGFloat
    ) -> CompositionHypothesis {
        // Aim Point Formula: P_target = (0.5 + (S_anchor.x - P_grid.x), 0.5 + (S_anchor.y - P_grid.y))
        let targetX = 0.5 + (anchor.x - gridPoint.x)
        let targetY = 0.5 + (anchor.y - gridPoint.y)
        let aimPoint = CGPoint(x: targetX, y: targetY)

        // Evaluate Score Components
        var score: Double = 50.0

        // 1. Proximity bonus (less camera rotation needed is preferred)
        let distFromCurrent = hypot(anchor.x - gridPoint.x, anchor.y - gridPoint.y)
        score += max(0.0, 30.0 * (1.0 - Double(distFromCurrent) * 1.5))

        // 2. Looking Room (Lead Room) Bonus:
        // If subject looks right (gaze.dx > 0), place subject on left (gridPoint.x < 0.5)
        if gaze.dx > 0.15 {
            if gridPoint.x < 0.5 {
                score += 25.0 * Double(gaze.dx)
            } else {
                score -= 20.0 * Double(gaze.dx)
            }
        } else if gaze.dx < -0.15 {
            // Subject looks left, place on right
            if gridPoint.x > 0.5 {
                score += 25.0 * Double(-gaze.dx)
            } else {
                score -= 20.0 * Double(-gaze.dx)
            }
        }

        // 3. Headroom Fit:
        // Ideal portrait headroom is between 10% and 15% (0.10 to 0.15) of the upper frame
        let projectedTopY = gridPoint.y - (anchor.y - subjectBox.minY)
        let idealHeadroom = 0.12
        let headroomError = abs(projectedTopY - idealHeadroom)
        if headroomError < 0.08 {
            score += 15.0 * (1.0 - Double(headroomError) / 0.08)
        } else {
            score -= 10.0 * Double(headroomError)
        }

        // 4. Rule-specific bonuses
        switch rule {
        case .goldenRatio:
            score += 5.0 // Slight preference for classical aesthetic
        case .centerSymmetry:
            // Symmetrical composition is only favorable when subject is near horizontal center
            if abs(anchor.x - 0.5) < 0.08 {
                score += 10.0
            } else {
                score -= 15.0
            }
        default:
            break
        }

        // 5. Virtual Crop Simulation for optimal zoom
        let recommendedZoom = calculateOptimalZoom(subjectBox: subjectBox, currentZoom: currentZoom)

        return CompositionHypothesis(
            rule: rule,
            gridPoint: gridPoint,
            aimPoint: aimPoint,
            score: max(0.0, score),
            recommendedZoom: recommendedZoom
        )
    }

    // MARK: - Virtual Crop Simulation (Smart Zoom Decision)
    public func calculateOptimalZoom(subjectBox: CGRect, currentZoom: CGFloat) -> CGFloat {
        // Compute subject area ratio rho = Area(S_box) / Area(Frame)
        let areaRatio = subjectBox.width * subjectBox.height

        var bestZoom: CGFloat = 1.0
        var bestScore: Double = -1.0

        for candidateZoom in availableZoomFactors {
            // Subject area if cropped/zoomed to candidateZoom
            let zoomScale = candidateZoom / max(0.5, currentZoom)
            let projectedArea = areaRatio * (zoomScale * zoomScale)
            let projectedHeight = subjectBox.height * zoomScale

            // Score based on ideal framing proportion:
            // Ideal portrait height should occupy between 45% and 70% of frame
            var zoomScore: Double = 0.0

            if projectedHeight > 0.90 || projectedArea > 0.70 {
                // Too close, cutting off subject body or context
                zoomScore = 0.0
            } else if projectedHeight < 0.20 {
                // Too far, subject lost in background
                zoomScore = 20.0
            } else {
                // High aesthetic comfort zone
                let heightDiff = abs(projectedHeight - 0.55)
                zoomScore = 100.0 - Double(heightDiff * 120.0)
            }

            // Slight penalty for extreme optical jumps to maintain stability
            if candidateZoom != currentZoom {
                zoomScore -= 5.0
            }

            if zoomScore > bestScore {
                bestScore = zoomScore
                bestZoom = candidateZoom
            }
        }

        return bestZoom
    }
}
