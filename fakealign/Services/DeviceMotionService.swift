import Foundation
import CoreMotion
import Combine

/// DeviceMotionService captures 200Hz IMU attitude updates with .xArbitraryZVertical reference frame
public final class DeviceMotionService: ObservableObject {
    public static let shared = DeviceMotionService()

    private let motionManager = CMMotionManager()
    private let motionQueue = OperationQueue()

    @Published public private(set) var currentRollDegrees: Double = 0.0
    @Published public private(set) var currentPitchDegrees: Double = 0.0
    @Published public private(set) var currentYawDegrees: Double = 0.0
    @Published public private(set) var isLevel: Bool = false
    @Published public private(set) var isRunning: Bool = false

    private var cancellables = Set<AnyCancellable>()

    private init() {
        motionQueue.name = "com.fakealign.motion.imu200HzQueue"
        motionQueue.qualityOfService = .userInteractive
        motionQueue.maxConcurrentOperationCount = 1
    }

    public func startUpdates() {
        guard motionManager.isDeviceMotionAvailable else {
            print("[DeviceMotionService] Device motion is not available on this hardware.")
            return
        }

        if isRunning { return }

        // Set 200Hz sampling interval (5ms per update)
        motionManager.deviceMotionUpdateInterval = 1.0 / 200.0
        motionManager.showsDeviceMovementDisplay = false

        // .xArbitraryZVertical provides gravity-aligned Z-axis with arbitrary horizontal reference
        motionManager.startDeviceMotionUpdates(using: .xArbitraryZVertical, to: motionQueue) { [weak self] motion, error in
            guard let self = self, let motion = motion, error == nil else { return }

            let timestamp = motion.timestamp
            let q = motion.attitude.quaternion

            // 1. Push directly into C++ Native Core ring buffer (lock-free, zero allocation)
            ntc_imu_push_sample(timestamp, q.x, q.y, q.z, q.w)

            // 2. Compute roll in degrees for the Virtual Horizon Leveler
            let rollRad = motion.attitude.roll
            let pitchRad = motion.attitude.pitch
            let yawRad = motion.attitude.yaw

            let rollDeg = rollRad * 180.0 / .pi
            let pitchDeg = pitchRad * 180.0 / .pi
            let yawDeg = yawRad * 180.0 / .pi

            // Perfectly level horizon when |roll| < 0.5 degrees
            let levelState = abs(rollDeg) < 0.5

            DispatchQueue.main.async {
                self.currentRollDegrees = rollDeg
                self.currentPitchDegrees = pitchDeg
                self.currentYawDegrees = yawDeg
                self.isLevel = levelState
            }
        }

        DispatchQueue.main.async {
            self.isRunning = true
        }
    }

    public func stopUpdates() {
        guard isRunning else { return }
        motionManager.stopDeviceMotionUpdates()
        ntc_imu_reset()

        DispatchQueue.main.async {
            self.isRunning = false
            self.isLevel = false
        }
    }
}
