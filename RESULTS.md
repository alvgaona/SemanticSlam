# Evaluation Results

## Setup

- **VIO**: OpenVINS with calibrated equidistant fisheye model (averaged across 5 flights)
- **SLAM**: G2O graph optimization with known gate positions as fixed landmarks
- **Detections**: Synthetic gate detections from VIO poses (recomputed with correct timestamps)
- **Evaluation**: `evo_ape` with SE(3) Umeyama alignment, and custom `evaluate_slam.py` with per-flight
  earth_to_map SVD calibration (skip=1000 VIO samples)

## Single-Graph Results

Detections routed directly to main graph. Throttled to 1 detection per gate per main keyframe.

### evo_ape (SE(3) Umeyama alignment)

| Flight | VIO RMSE (m) | Corrected RMSE (m) | Improvement |
|--------|-------------|--------------------|-----------:|
| 03p    | 0.729       | 0.666              |      +8.6% |

### evaluate_slam.py (per-flight earth_to_map calibration)

| Flight | Metric | VIO RMSE (m) | Corrected RMSE (m) | Improvement |
|--------|--------|-------------|--------------------|-----------:|
| 01p    | 3D     | 1.636       | 1.292              |     +21.0% |
| 01p    | XY     | 1.333       | 1.093              |     +18.0% |
| 01p    | Z      | 0.949       | 0.689              |     +27.4% |
| 02p    | 3D     | 0.740       | 0.648              |     +12.5% |
| 02p    | XY     | 0.648       | 0.584              |      +9.9% |
| 02p    | Z      | 0.358       | 0.281              |     +21.5% |
| 03p    | 3D     | 1.010       | 0.788              |     +22.0% |
| 03p    | XY     | 0.886       | 0.664              |     +25.1% |
| 03p    | Z      | 0.486       | 0.425              |     +12.5% |

### Summary (evaluate_slam.py, 3D RMSE)

| Flight | VIO (m) | Corrected (m) | Improvement |
|--------|---------|---------------|-----------:|
| 01p    | 1.636   | 1.292         |     +21.0% |
| 02p    | 0.740   | 0.648         |     +12.5% |
| 03p    | 1.010   | 0.788         |     +22.0% |

## Dual-Graph Results

Detections routed to temp graph, optimized, then merged to main graph with body-relative measurements.

### evaluate_slam.py (per-flight earth_to_map calibration)

| Flight | Metric | VIO RMSE (m) | Corrected RMSE (m) | Improvement |
|--------|--------|-------------|--------------------|-----------:|
| 02p    | 3D     | 0.740       | 0.851              |     -14.9% |
| 02p    | XY     | 0.648       | 0.793              |     -22.3% |
| 02p    | Z      | 0.358       | 0.309              |     +13.8% |

Dual-graph is worse than raw VIO in XY. The temp graph optimization introduces bias in the extracted
body-relative measurement compared to direct sensor observations.

## Notes

- **evo vs evaluate_slam.py**: evo uses optimal SE(3) Umeyama alignment (best possible alignment for both
  trajectories). evaluate_slam.py uses a fixed earth_to_map calibration per flight. evo gives a stricter,
  more conservative improvement number.
- **Per-flight calibration**: earth_to_map must be computed per flight because OpenVINS initializes with
  an arbitrary heading. The calibration script (`scripts/calibrate_earth_to_map.py`) must skip the first
  ~1000 VIO samples to avoid unconverged estimates.
- **Throttling**: Single-graph throttles to 1 detection per gate per main keyframe. Without throttling,
  the graph is overwhelmed with redundant constraints and performance degrades.
- **Detections are synthetic**: Generated from VIO poses and known gate positions (zero noise). Real
  vision-based detections will have noise, missed detections, and false positives.
