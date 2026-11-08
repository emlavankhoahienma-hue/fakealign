import Foundation
import simd
import Combine

extension simd_quatd {
    public func act(_ v: simd_double3) -> simd_double3 {
        return simd_act(self, v)
    }
}

/// Measurement classification for adaptive Kalman filtering
public enum SpatialMeasurementType {
    case confirmedContinuation // High confidence optical tracking continuation
    case verifiedContinuation  // Verified feature match continuation
    case exploratory           // Saliency or initial candidate
}

/// 3D unit ray representing a directional bearing on the unit sphere S^2
public struct S2Ray: Equatable {
    public var vector: simd_double3

    public init(vector: simd_double3) {
        let len = simd_length(vector)
        if len > 1e-12 {
            self.vector = vector / len
        } else {
            self.vector = simd_double3(0, 0, 1)
        }
    }

    public init(x: Double, y: Double, z: Double) {
        self.init(vector: simd_double3(x, y, z))
    }

    public static let forward = S2Ray(vector: simd_double3(0, 0, 1))
}

/// Error-State Kalman Filter on the tangent plane of S^2
public final class TangentPlaneESKF {
    public private(set) var nominalRay: S2Ray
    private var e1: simd_double3 = simd_double3(1, 0, 0)
    private var e2: simd_double3 = simd_double3(0, 1, 0)

    // State vector: [dtheta1, dtheta2, omega1, omega2]^T
    private var state: simd_double4 = simd_double4(0, 0, 0, 0)
    // Covariance 4x4
    private var P: (simd_double4, simd_double4, simd_double4, simd_double4)
    private var lastTimestamp: Double = 0.0

    public init(initialRay: S2Ray, timestamp: Double) {
        self.nominalRay = initialRay
        self.lastTimestamp = timestamp
        self.P = (
            simd_double4(0.01, 0, 0, 0),
            simd_double4(0, 0.01, 0, 0),
            simd_double4(0, 0, 0.05, 0),
            simd_double4(0, 0, 0, 0.05)
        )
        updateTangentBasis()
    }

    private func updateTangentBasis() {
        let r = nominalRay.vector
        if abs(r.z) < 0.9 {
            e1 = simd_normalize(simd_cross(r, simd_double3(0, 0, 1)))
        } else {
            e1 = simd_normalize(simd_cross(r, simd_double3(1, 0, 0)))
        }
        e2 = simd_normalize(simd_cross(r, e1))
    }

    public func predict(to timestamp: Double) {
        let dt = max(1e-4, min(0.1, timestamp - lastTimestamp))
        lastTimestamp = timestamp

        // Handheld angular damping: omega_{t+dt} = omega_t * exp(-2.85 * dt)
        let dampingFactor = exp(-2.85 * dt)
        state.z *= dampingFactor
        state.w *= dampingFactor

        // Position error prediction: dtheta += omega * dt
        state.x += state.z * dt
        state.y += state.w * dt

        // Process noise
        let qPos = 0.0001 * dt
        let qVel = 0.005 * dt
        P.0.x += qPos
        P.1.y += qPos
        P.2.z += qVel
        P.3.w += qVel
    }

    public func update(observedRay: S2Ray, confidence: Double, type: SpatialMeasurementType) -> (accepted: Bool, residual: Double) {
        let rObs = observedRay.vector
        let rNom = nominalRay.vector

        // Residual in tangent space (y = [e1 . (rObs - rNom), e2 . (rObs - rNom)])
        let diff = rObs - rNom
        let y = simd_double2(simd_dot(e1, diff), simd_dot(e2, diff))
        let residualNorm = simd_length(y)

        // Reject severe outlier jumps
        if residualNorm > 0.30 {
            return (false, residualNorm)
        }

        // Measurement noise variance inversely proportional to optical confidence
        let rVar = max(0.0004, 0.04 * (1.0 - confidence) + 0.001)
        let S00 = P.0.x + rVar
        let S11 = P.1.y + rVar
        let detS = S00 * S11

        if detS <= 1e-12 {
            return (false, residualNorm)
        }

        // Mahalanobis distance squared: d_M^2 = y^T S^-1 y
        let dM2 = (y.x * y.x / S00) + (y.y * y.y / S11)

        // Adaptive gating check
        let gateThreshold: Double = {
            switch type {
            case .confirmedContinuation: return 144.0
            case .verifiedContinuation:  return 64.0
            case .exploratory:           return 25.0
            }
        }()

        if dM2 > gateThreshold {
            return (false, residualNorm)
        }

        // Kalman gain K = P * H^T * S^-1
        let k0 = P.0.x / S00
        let k1 = P.1.y / S11
        let k2 = P.2.x / S00
        let k3 = P.3.y / S11

        state.x += k0 * y.x
        state.y += k1 * y.y
        state.z += k2 * y.x
        state.w += k3 * y.y

        // Inject error into nominal state
        let correctedVector = rNom + state.x * e1 + state.y * e2
        nominalRay = S2Ray(vector: correctedVector)
        state.x = 0
        state.y = 0
        updateTangentBasis()

        // Covariance update: (I - KH)P
        P.0.x *= (1.0 - k0)
        P.1.y *= (1.0 - k1)

        return (true, residualNorm)
    }

    public func reidentify(newRay: S2Ray, timestamp: Double) {
        self.nominalRay = newRay
        self.lastTimestamp = timestamp
        self.state = simd_double4(0, 0, 0, 0)
        self.P = (
            simd_double4(0.01, 0, 0, 0),
            simd_double4(0, 0.01, 0, 0),
            simd_double4(0, 0.05, 0, 0),
            simd_double4(0, 0, 0, 0.05)
        )
        updateTangentBasis()
    }
}

/// SpatialTrackingEngine coordinates 3D ray tracking on S^2 with 0ms visual latency
public final class SpatialTrackingEngine: ObservableObject {
    public static let shared = SpatialTrackingEngine()

    // Real physical subject bearing ray in world space (e.g. human face, tree top)
    @Published public private(set) var subjectWorldRay: S2Ray = .forward
    // Artistic guide bearing ray in world space (aim point for target composition)
    @Published public private(set) var worldRay: S2Ray = .forward
    // Viewfinder presentation ray filtered via 1-Euro Geodesic SLERP on S^2
    @Published public private(set) var presentationGuideRay: S2Ray = .forward

    // Fixed quaternion offset between subject and guide ray: r_guide = q_offset * r_subject
    private var guideFromSubject: simd_quatd = simd_quatd(real: 1.0, imag: simd_double3(0, 0, 0))

    // Filtering & Kalman Engine
    private var eskf: TangentPlaneESKF?
    @Published public private(set) var isTrackingActive: Bool = false
    @Published public private(set) var presentationIsRecovering: Bool = false

    private var lastPresentationTime: Double = 0.0
    private let engineLock = NSLock()

    private init() {}

    // MARK: - Lock Target Ray
    public func lockTarget(subjectRay: S2Ray, guideRay: S2Ray, timestamp: Double) {
        engineLock.lock()
        defer { engineLock.unlock() }

        self.subjectWorldRay = subjectRay
        self.worldRay = guideRay
        self.presentationGuideRay = guideRay

        // Compute rotation quaternion q that maps subjectRay to guideRay
        let vFrom = subjectRay.vector
        let vTo = guideRay.vector
        let dot = simd_clamp(simd_dot(vFrom, vTo), -1.0, 1.0)

        if dot > 0.999999 {
            self.guideFromSubject = simd_quatd(real: 1.0, imag: simd_double3(0, 0, 0))
        } else if dot < -0.999999 {
            let axis = simd_normalize(abs(vFrom.z) < 0.9 ? simd_cross(vFrom, simd_double3(0, 0, 1)) : simd_cross(vFrom, simd_double3(1, 0, 0)))
            self.guideFromSubject = simd_quatd(angle: .pi, axis: axis)
        } else {
            let cross = simd_cross(vFrom, vTo)
            let q = simd_quatd(real: 1.0 + dot, imag: cross)
            self.guideFromSubject = simd_normalize(q)
        }

        self.eskf = TangentPlaneESKF(initialRay: subjectRay, timestamp: timestamp)
        self.lastPresentationTime = timestamp
        self.isTrackingActive = true
        self.presentationIsRecovering = false
    }

    public func unlockTarget() {
        engineLock.lock()
        defer { engineLock.unlock() }

        self.isTrackingActive = false
        self.eskf = nil
        self.presentationIsRecovering = false
    }

    // MARK: - Optical Measurement Ingestion
    public func ingestOpticalMeasurement(observedSubjectRay: S2Ray,
                                         confidence: Double,
                                         timestamp: Double,
                                         type: SpatialMeasurementType) {
        engineLock.lock()
        defer { engineLock.unlock() }

        guard isTrackingActive, let filter = eskf else { return }

        // Predict step
        filter.predict(to: timestamp)

        // Parallax Fast Re-ID check:
        // When user takes a step, parallax shift causes optical residual > 0.08 while optical confidence is high (>= 0.65)
        let diff = simd_length(observedSubjectRay.vector - filter.nominalRay.vector)
        if diff > 0.08 && confidence >= 0.65 {
            filter.reidentify(newRay: observedSubjectRay, timestamp: timestamp)
            self.subjectWorldRay = observedSubjectRay
            self.worldRay = S2Ray(vector: guideFromSubject.act(observedSubjectRay.vector))
            self.presentationIsRecovering = true
            return
        }

        // ESKF update
        let (accepted, _) = filter.update(observedRay: observedSubjectRay, confidence: confidence, type: type)
        if accepted {
            self.subjectWorldRay = filter.nominalRay
            self.worldRay = S2Ray(vector: guideFromSubject.act(filter.nominalRay.vector))
            self.presentationIsRecovering = false
        }
    }

    // MARK: - Viewfinder Latency Compensation & 1-Euro SLERP
    public func updateViewfinderPresentation(currentTimestamp: Double, focalScale: Double) -> S2Ray {
        engineLock.lock()
        defer { engineLock.unlock() }

        guard isTrackingActive else {
            return .forward
        }

        let dt = max(1e-4, min(0.05, currentTimestamp - lastPresentationTime))
        lastPresentationTime = currentTimestamp

        var outX: Double = 0
        var outY: Double = 0
        var outZ: Double = 0

        // Call C++ NativeCore 1-Euro Geodesic SLERP on S^2
        ntc_s2_1euro_slerp(
            presentationGuideRay.vector.x,
            presentationGuideRay.vector.y,
            presentationGuideRay.vector.z,
            worldRay.vector.x,
            worldRay.vector.y,
            worldRay.vector.z,
            focalScale,
            dt,
            &outX,
            &outY,
            &outZ
        )

        let smoothedRay = S2Ray(x: outX, y: outY, z: outZ)
        self.presentationGuideRay = smoothedRay
        return smoothedRay
    }

    // MARK: - Ray to Camera Projection
    public func projectRayToCameraScreen(ray: S2Ray, cameraAttitude: simd_quatd, fovHorizontalRad: Double, fovVerticalRad: Double) -> CGPoint {
        // Transform world ray to current camera coordinate frame: r_cam = q_cam^-1 * r_world
        let rCam = cameraAttitude.inverse.act(ray.vector)

        // Point is behind camera
        if rCam.z <= 0.01 {
            let normX = rCam.x >= 0 ? 2.0 : -1.0
            let normY = rCam.y >= 0 ? 2.0 : -1.0
            return CGPoint(x: normX, y: normY)
        }

        let tanHalfH = tan(fovHorizontalRad * 0.5)
        let tanHalfV = tan(fovVerticalRad * 0.5)

        let screenX = 0.5 + (rCam.x / rCam.z) / (2.0 * tanHalfH)
        let screenY = 0.5 - (rCam.y / rCam.z) / (2.0 * tanHalfV)

        return CGPoint(x: screenX, y: screenY)
    }

    // MARK: - Camera Screen to World Ray
    public func screenPointToWorldRay(point: CGPoint, cameraAttitude: simd_quatd, fovHorizontalRad: Double, fovVerticalRad: Double) -> S2Ray {
        let tanHalfH = tan(fovHorizontalRad * 0.5)
        let tanHalfV = tan(fovVerticalRad * 0.5)

        let rx = (point.x - 0.5) * (2.0 * tanHalfH)
        let ry = -(point.y - 0.5) * (2.0 * tanHalfV)
        let rz = 1.0

        let rayCam = simd_normalize(simd_double3(rx, ry, rz))
        let rayWorld = cameraAttitude.act(rayCam)

        return S2Ray(vector: rayWorld)
    }
}
