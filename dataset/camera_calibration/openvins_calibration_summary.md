# OpenVINS Online Calibration Results — Piloted Ellipse Flights

Camera: ArduCam IMX219, 175° diagonal FOV fisheye, 640x480 @ 120fps
IMU: SBG Ellipse, 500Hz
Model: equidistant (Kannala-Brandt) fisheye

## Configuration

- OpenVINS with online calibration: extrinsics + intrinsics + time offset
- IMU noise: accel_noise=4e-2, accel_rw=6e-3, gyro_noise=4e-3, gyro_rw=2e-4
- Track frequency: 30Hz (flights 01p-04p with 150 pts, 05p-06p with 300 pts)
- Initial params: flight-01p started from FOV-derived guess, all others from flight-02p converged values

## Per-Flight Results

### Intrinsics [fx, fy, cx, cy]

| Flight | fx      | fy      | cx      | cy      |
|--------|---------|---------|---------|---------|
| 01p*   | 265.249 | 340.573 | 318.979 | 257.933 |
| 02p    | 287.707 | 383.498 | 317.957 | 243.668 |
| 03p    | 292.652 | 392.389 | 315.910 | 241.023 |
| 04p    | 292.439 | 391.000 | 316.537 | 236.297 |
| 05p    | 290.715 | 389.767 | 317.249 | 242.889 |
| 06p    | 294.088 | 393.402 | 314.581 | 238.335 |

*01p excluded from averages (poor initial guess caused slower convergence)

### Distortion (equidistant) [k1, k2, k3, k4]

| Flight | k1     | k2     | k3     | k4     |
|--------|--------|--------|--------|--------|
| 01p*   | 0.037  | -0.012 | 0.024  | -0.011 |
| 02p    | 0.047  | -0.025 | 0.031  | -0.013 |
| 03p    | 0.045  | -0.027 | 0.038  | -0.019 |
| 04p    | 0.049  | -0.030 | 0.040  | -0.020 |
| 05p    | 0.052  | -0.034 | 0.042  | -0.020 |
| 06p    | 0.046  | -0.025 | 0.037  | -0.020 |

### Extrinsics — Quaternion JPL (x, y, z, w)

| Flight | qx     | qy    | qz     | qw    |
|--------|--------|-------|--------|-------|
| 01p*   | -0.381 | 0.372 | -0.601 | 0.596 |
| 02p    | -0.366 | 0.369 | -0.610 | 0.599 |
| 03p    | -0.365 | 0.365 | -0.611 | 0.600 |
| 04p    | -0.361 | 0.365 | -0.613 | 0.601 |
| 05p    | -0.364 | 0.366 | -0.611 | 0.600 |
| 06p    | -0.364 | 0.363 | -0.611 | 0.602 |

### Extrinsics — Translation p_IinC

| Flight | x      | y      | z      |
|--------|--------|--------|--------|
| 01p*   | -0.131 | 0.309  | -0.425 |
| 02p    | 0.004  | 0.032  | -0.129 |
| 03p    | 0.032  | -0.001 | -0.041 |
| 04p    | 0.038  | 0.048  | -0.036 |
| 05p    | 0.032  | -0.005 | -0.072 |
| 06p    | 0.036  | 0.059  | -0.024 |

### Camera-IMU Time Offset

| Flight | offset (s) |
|--------|------------|
| 01p*   | -0.00924   |
| 02p    | -0.01037   |
| 03p    | -0.01354   |
| 04p    | -0.01035   |
| 05p    | -0.01366   |
| 06p    | -0.01074   |

## Mean and Standard Deviation (flights 02p–06p)

### Intrinsics

| Param | Mean    | Std   |
|-------|---------|-------|
| fx    | 291.520 | 2.187 |
| fy    | 390.011 | 3.482 |
| cx    | 316.447 | 1.158 |
| cy    | 240.442 | 2.770 |

### Distortion (equidistant)

| Param | Mean   | Std    |
|-------|--------|--------|
| k1    | 0.048  | 0.0025 |
| k2    | -0.028 | 0.0034 |
| k3    | 0.038  | 0.0037 |
| k4    | -0.018 | 0.0027 |

### Extrinsics — Quaternion JPL (x, y, z, w)

| Param | Mean   | Std    |
|-------|--------|--------|
| qx    | -0.364 | 0.0017 |
| qy    | 0.366  | 0.0020 |
| qz    | -0.611 | 0.0010 |
| qw    | 0.600  | 0.0010 |

### Extrinsics — Translation p_IinC

| Param | Mean   | Std   |
|-------|--------|-------|
| x     | 0.028  | 0.012 |
| y     | 0.027  | 0.027 |
| z     | -0.060 | 0.038 |

### Camera-IMU Time Offset

| Param      | Mean     | Std     |
|------------|----------|---------|
| time_off   | -0.01173 | 0.00153 |

## Notes

- The original TII-RATM calibration (cv2.calibrateCamera with radtan model) gives fx=289.88, fy=387.06,
  which is within 1 std of the equidistant model mean (291.5, 390.0). The raw calibration intrinsics
  were approximately correct; the issue was using them with the wrong distortion model.
- The extrinsics quaternion is extremely consistent (std < 0.002), confirming a fixed camera mount.
- The translation p_IinC has higher variance, typical for monocular VIO where translation is less
  observable than rotation.
- Flight 01p was excluded from averages because it started from a poor initial guess (0° tilt,
  FOV-derived intrinsics), causing slower/incomplete convergence.
