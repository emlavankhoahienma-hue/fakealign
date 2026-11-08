# FakeAlign - AI Smart Framing Camera

> 3D Spatial Target Tracking & Autonomous Photographic Composition Engine for iOS 17+.

## Key Features & Architecture

1. **Native C++20 Core & Metal Acceleration**:
   - `NativeTrackingCore`: Lock-free circular ring buffer (1024 samples) capturing 200Hz CoreMotion updates with 0.0ms latency.
   - `TargetPatchFlowShaders.metal`: GPU-accelerated Shi-Tomasi minimum eigenvalue corner detection and Pyramidal Lucas-Kanade optical flow.
   - Geodesic SLERP 1-Euro Filter on the unit sphere $S^2$ for jitter-free handheld tracking.

2. **Apple Native Frameworks (Zero CoreML Debug Friction)**:
   - `AVFoundation`: 60 FPS live preview, camera intrinsics delivery, smart zoom ramping, AE/AF point of interest, and exposure bias control.
   - `CoreMotion`: 200Hz IMU attitude capture with `.xArbitraryZVertical` reference frame.
   - `Vision`: Continuous tracking via `VNTrackObjectRequest`, portrait eye anchoring via `VNDetectFaceLandmarksRequest`, human body pose detection via `VNDetectHumanBodyPoseRequest`.
   - `2-Tier Re-ID`: Fast Saliency Scan (~7Hz) + 25-cell Neural FeaturePrint Grid Search (`VNGenerateImageFeaturePrintRequest`).

3. **Spatial Tracking & Math Engine**:
   - Unit bearing ray $\mathbf{r} \in S^2, \|\mathbf{r}\| = 1$.
   - Local Tangent Plane Error-State Kalman Filter (ESKF) with handheld damping $\boldsymbol{\omega}_{t+\Delta t} = \boldsymbol{\omega}_t \cdot \exp(-2.85 \Delta t)$.
   - Adaptive Mahalanobis Gate & Parallax Fast Re-ID when walking/stepping.
   - Viewfinder latency lead compensation (~8ms).

4. **Multi-Hypothesis Photographic Framing**:
   - Compares Rule of Thirds, Golden Ratio / Phi Grid, Center Symmetry, and Leading Lines.
   - Aim Point Formula:
     $$P_{target} = \left(0.5 + (S_{anchor}.x - P_{grid}.x), \; 0.5 + (S_{anchor}.y - P_{grid}.y)\right)$$
   - Gaze Looking Room bonus and Golden Ratio Headroom fit.
   - Virtual Crop Simulation: area ratio $\rho = \text{Area}(S_{box}) / \text{Area}(Frame)$ automatically selecting $0.5\times, 1\times, 2\times, 5\times$ zoom.

5. **SwiftUI AR Viewfinder UI**:
   - 3D Gold Aim Reticle (`#FFD700`) with micro crosshairs.
   - Edge Docking with directional navigation arrows when off-screen.
   - Center Alignment Ring with glowing green aura upon lock.
   - Virtual Horizon Leveler ($|\text{roll}| < 0.5^\circ$).
   - Interactive Sun Exposure Slider ($-2.0$ EV to $+2.0$ EV).

6. **Automated CI/CD Unsigned IPA**:
   - GitHub Actions workflow (`.github/workflows/ios-build.yml`) builds `.xcarchive` without codesigning (`CODE_SIGNING_ALLOWED=NO`).
   - Generates and uploads `fakealign-unsigned.ipa` ready to install via Sideloadly, TrollStore, or AltStore.

## Project Structure

```
fakealign/
├── .github/workflows/
│   └── ios-build.yml
├── fakealign.xcodeproj/
│   ├── project.pbxproj
│   └── xcshareddata/xcschemes/fakealign.xcscheme
├── fakealign/
│   ├── App/
│   │   └── FakeAlignApp.swift
│   ├── Services/
│   │   ├── NativeTrackingCore.h
│   │   ├── NativeTrackingCore.cpp
│   │   ├── CameraService.swift
│   │   ├── DeviceMotionService.swift
│   │   ├── SpatialTrackingEngine.swift
│   │   ├── VisionFramingEngine.swift
│   │   └── CompositionCalculator.swift
│   ├── ViewModels/
│   │   └── CameraViewModel.swift
│   ├── Views/
│   │   ├── ContentView.swift
│   │   ├── CameraPreviewView.swift
│   │   └── ARFramingOverlayView.swift
│   ├── Shaders/
│   │   └── TargetPatchFlowShaders.metal
│   ├── fakealign-Bridging-Header.h
│   └── Info.plist
├── .gitignore
└── README.md
```
