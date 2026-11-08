#include "NativeTrackingCore.h"
#include <cmath>
#include <atomic>
#include <array>
#include <algorithm>

namespace {

constexpr size_t kRingBufferSize = 1024;
constexpr size_t kRingBufferMask = kRingBufferSize - 1;
constexpr double kPi = 3.14159265358979323846;

struct alignas(64) RingBuffer {
    std::array<NTCMotionSample, kRingBufferSize> samples{};
    std::atomic<uint64_t> writeSequence{0};
};

RingBuffer g_motionBuffer;

inline void normalizeQuaternion(double& x, double& y, double& z, double& w) {
    double norm = std::sqrt(x * x + y * y + z * z + w * w);
    if (norm > 1e-12) {
        double invNorm = 1.0 / norm;
        x *= invNorm;
        y *= invNorm;
        z *= invNorm;
        w *= invNorm;
    } else {
        x = 0.0;
        y = 0.0;
        z = 0.0;
        w = 1.0;
    }
}

inline void slerpQuaternion(double q0x, double q0y, double q0z, double q0w,
                            double q1x, double q1y, double q1z, double q1w,
                            double t,
                            double& outX, double& outY, double& outZ, double& outW) {
    double dot = q0x * q1x + q0y * q1y + q0z * q1z + q0w * q1w;

    if (dot < 0.0) {
        dot = -dot;
        q1x = -q1x;
        q1y = -q1y;
        q1z = -q1z;
        q1w = -q1w;
    }

    if (dot > 0.9995) {
        // NLERP for nearly collinear orientations
        outX = q0x + t * (q1x - q0x);
        outY = q0y + t * (q1y - q0y);
        outZ = q0z + t * (q1z - q0z);
        outW = q0w + t * (q1w - q0w);
        normalizeQuaternion(outX, outY, outZ, outW);
        return;
    }

    dot = std::clamp(dot, -1.0, 1.0);
    double theta0 = std::acos(dot);
    double sinTheta0 = std::sin(theta0);
    double theta = theta0 * t;
    double sinTheta = std::sin(theta);

    double s0 = std::cos(theta) - dot * sinTheta / sinTheta0;
    double s1 = sinTheta / sinTheta0;

    outX = (s0 * q0x) + (s1 * q1x);
    outY = (s0 * q0y) + (s1 * q1y);
    outZ = (s0 * q0z) + (s1 * q1z);
    outW = (s0 * q0w) + (s1 * q1w);
    normalizeQuaternion(outX, outY, outZ, outW);
}

} // anonymous namespace

extern "C" {

void ntc_imu_push_sample(double timestamp, double qx, double qy, double qz, double qw) {
    normalizeQuaternion(qx, qy, qz, qw);

    uint64_t seq = g_motionBuffer.writeSequence.load(std::memory_order_relaxed);
    size_t index = seq & kRingBufferMask;

    g_motionBuffer.samples[index] = NTCMotionSample{timestamp, qx, qy, qz, qw};
    g_motionBuffer.writeSequence.store(seq + 1, std::memory_order_release);
}

bool ntc_imu_interpolate_pose(double timestamp, double* qx, double* qy, double* qz, double* qw) {
    if (!qx || !qy || !qz || !qw) {
        return false;
    }

    uint64_t currentSeq = g_motionBuffer.writeSequence.load(std::memory_order_acquire);
    if (currentSeq == 0) {
        return false;
    }

    size_t availableCount = (currentSeq > kRingBufferSize) ? kRingBufferSize : static_cast<size_t>(currentSeq);
    uint64_t oldestSeq = currentSeq - availableCount;
    uint64_t newestSeq = currentSeq - 1;

    const NTCMotionSample& oldestSample = g_motionBuffer.samples[oldestSeq & kRingBufferMask];
    const NTCMotionSample& newestSample = g_motionBuffer.samples[newestSeq & kRingBufferMask];

    // Boundary conditions
    if (timestamp <= oldestSample.timestamp) {
        *qx = oldestSample.qx;
        *qy = oldestSample.qy;
        *qz = oldestSample.qz;
        *qw = oldestSample.qw;
        return true;
    }

    if (timestamp >= newestSample.timestamp) {
        *qx = newestSample.qx;
        *qy = newestSample.qy;
        *qz = newestSample.qz;
        *qw = newestSample.qw;
        return true;
    }

    // Binary search for interval [t_low, t_high]
    uint64_t low = oldestSeq;
    uint64_t high = newestSeq;

    while (low + 1 < high) {
        uint64_t mid = low + (high - low) / 2;
        if (g_motionBuffer.samples[mid & kRingBufferMask].timestamp <= timestamp) {
            low = mid;
        } else {
            high = mid;
        }
    }

    const NTCMotionSample& s0 = g_motionBuffer.samples[low & kRingBufferMask];
    const NTCMotionSample& s1 = g_motionBuffer.samples[high & kRingBufferMask];

    double dt = s1.timestamp - s0.timestamp;
    if (dt <= 1e-9) {
        *qx = s1.qx;
        *qy = s1.qy;
        *qz = s1.qz;
        *qw = s1.qw;
        return true;
    }

    double t = std::clamp((timestamp - s0.timestamp) / dt, 0.0, 1.0);
    slerpQuaternion(s0.qx, s0.qy, s0.qz, s0.qw,
                    s1.qx, s1.qy, s1.qz, s1.qw,
                    t,
                    *qx, *qy, *qz, *qw);
    return true;
}

double ntc_imu_current_angular_speed(void) {
    uint64_t currentSeq = g_motionBuffer.writeSequence.load(std::memory_order_acquire);
    if (currentSeq < 2) {
        return 0.0;
    }

    const NTCMotionSample& s1 = g_motionBuffer.samples[(currentSeq - 1) & kRingBufferMask];
    const NTCMotionSample& s0 = g_motionBuffer.samples[(currentSeq - 2) & kRingBufferMask];

    double dt = s1.timestamp - s0.timestamp;
    if (dt <= 1e-6) {
        return 0.0;
    }

    double dot = std::abs(s0.qx * s1.qx + s0.qy * s1.qy + s0.qz * s1.qz + s0.qw * s1.qw);
    dot = std::clamp(dot, -1.0, 1.0);
    double deltaTheta = 2.0 * std::acos(dot);
    return deltaTheta / dt;
}

double ntc_imu_angular_displacement_since(double timestamp) {
    double pastQx, pastQy, pastQz, pastQw;
    if (!ntc_imu_interpolate_pose(timestamp, &pastQx, &pastQy, &pastQz, &pastQw)) {
        return 0.0;
    }

    uint64_t currentSeq = g_motionBuffer.writeSequence.load(std::memory_order_acquire);
    if (currentSeq == 0) {
        return 0.0;
    }

    const NTCMotionSample& current = g_motionBuffer.samples[(currentSeq - 1) & kRingBufferMask];
    double dot = std::abs(pastQx * current.qx + pastQy * current.qy + pastQz * current.qz + pastQw * current.qw);
    dot = std::clamp(dot, -1.0, 1.0);
    return 2.0 * std::acos(dot);
}

void ntc_s2_1euro_slerp(double prevX, double prevY, double prevZ,
                         double targetX, double targetY, double targetZ,
                         double focalScale, double dt,
                         double* outX, double* outY, double* outZ) {
    if (!outX || !outY || !outZ) {
        return;
    }

    // Normalize input vectors on S^2
    double pNorm = std::sqrt(prevX * prevX + prevY * prevY + prevZ * prevZ);
    if (pNorm > 1e-12) {
        double invP = 1.0 / pNorm;
        prevX *= invP;
        prevY *= invP;
        prevZ *= invP;
    } else {
        prevX = 0.0; prevY = 0.0; prevZ = 1.0;
    }

    double tNorm = std::sqrt(targetX * targetX + targetY * targetY + targetZ * targetZ);
    if (tNorm > 1e-12) {
        double invT = 1.0 / tNorm;
        targetX *= invT;
        targetY *= invT;
        targetZ *= invT;
    } else {
        targetX = 0.0; targetY = 0.0; targetZ = 1.0;
    }

    // Geodesic angle on S^2
    double crossX = prevY * targetZ - prevZ * targetY;
    double crossY = prevZ * targetX - prevX * targetZ;
    double crossZ = prevX * targetY - prevY * targetX;
    double crossNorm = std::sqrt(crossX * crossX + crossY * crossY + crossZ * crossZ);
    double dot = std::clamp(prevX * targetX + prevY * targetY + prevZ * targetZ, -1.0, 1.0);
    double theta = std::atan2(crossNorm, dot);

    if (dt <= 0.0) {
        dt = 1.0 / 60.0;
    }

    // Velocity-adaptive cutoff frequency
    double cutoffHz = 14.0 + std::min(85.0, theta * focalScale * 120.0);
    double alpha = 1.0 - std::exp(-2.0 * kPi * cutoffHz * dt);
    alpha = std::clamp(alpha, 0.0, 1.0);

    if (theta < 1e-7 || crossNorm < 1e-7) {
        *outX = targetX;
        *outY = targetY;
        *outZ = targetZ;
        return;
    }

    double sinTheta = std::sin(theta);
    double w1 = std::sin((1.0 - alpha) * theta) / sinTheta;
    double w2 = std::sin(alpha * theta) / sinTheta;

    double resX = w1 * prevX + w2 * targetX;
    double resY = w1 * prevY + w2 * targetY;
    double resZ = w1 * prevZ + w2 * targetZ;

    double rNorm = std::sqrt(resX * resX + resY * resY + resZ * resZ);
    if (rNorm > 1e-12) {
        double invR = 1.0 / rNorm;
        *outX = resX * invR;
        *outY = resY * invR;
        *outZ = resZ * invR;
    } else {
        *outX = targetX;
        *outY = targetY;
        *outZ = targetZ;
    }
}

void ntc_imu_reset(void) {
    g_motionBuffer.writeSequence.store(0, std::memory_order_release);
}

} // extern "C"
