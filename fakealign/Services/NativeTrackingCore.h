#ifndef NativeTrackingCore_h
#define NativeTrackingCore_h

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/// Motion sample received from 200Hz CoreMotion updates
typedef struct {
    double timestamp;
    double qx;
    double qy;
    double qz;
    double qw;
} NTCMotionSample;

/// Result of optical patch flow displacement and tracking reliability
typedef struct {
    double dx;
    double dy;
    double confidence;
    int inliers;
    bool isReliable;
} NTCPatchFlowResult;

/**
 * Pushes an IMU attitude sample into the lock-free circular ring buffer (200Hz).
 */
void ntc_imu_push_sample(double timestamp, double qx, double qy, double qz, double qw);

/**
 * Interpolates device attitude at a specific query timestamp using SLERP on quaternions.
 * Returns true if valid interpolation or closest boundary sample was retrieved.
 */
bool ntc_imu_interpolate_pose(double timestamp, double* qx, double* qy, double* qz, double* qw);

/**
 * Calculates the current angular speed in radians/second from the latest IMU samples.
 */
double ntc_imu_current_angular_speed(void);

/**
 * Calculates total geodesic angular displacement (in radians) between a past timestamp and now.
 */
double ntc_imu_angular_displacement_since(double timestamp);

/**
 * Velocity-adaptive Geodesic 1-Euro SLERP on the unit 2-sphere (S^2).
 * Prevents jitter while providing zero-latency tracking during fast camera movements.
 */
void ntc_s2_1euro_slerp(double prevX, double prevY, double prevZ,
                         double targetX, double targetY, double targetZ,
                         double focalScale, double dt,
                         double* outX, double* outY, double* outZ);

/**
 * Resets the internal IMU ring buffer state.
 */
void ntc_imu_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* NativeTrackingCore_h */
