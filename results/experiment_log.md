# Experiment Log

## Experiment 01 - Cumulative LS + Kalman Filter

### Setup

- AOA noise standard deviation: 2 deg
- Emitter position: (30, 30) m
- UAV start position: (0, 5) m
- UAV velocity: (2.0, 0.5) m/s
- Simulation time: 20 s
- Time step: 0.1 s
- Minimum AOA samples: 10

### Kalman Filter Parameters

- Initial covariance P: 10.0
- Process noise Q: 0.01
- Measurement noise R: 4.0
- State: [x, y]
- Stationary emitter model

### Results

- Raw mean localization error: 7.755 m
- Kalman mean localization error: 11.323 m

### Observation

Kalman filtering increased the mean localization error compared with the
cumulative least-squares estimate.

The cumulative least-squares measurement already uses all previous AOA
measurements, so consecutive position estimates are strongly correlated and
already smoothed.

The Kalman filter also gives limited weight to new measurements because of
the current measurement noise setting, causing the initial estimation error
to persist.

### Next Experiment

Replace cumulative least-squares measurements with rolling-window
least-squares measurements and compare raw and Kalman-filtered localization
errors again.

## Experiment 02 - Rolling LS + Kalman Filter

### Setup

- AOA noise standard deviation: 2 deg
- Rolling window size: 10
- Emitter position: (30, 30) m
- UAV start position: (0, 5) m
- UAV velocity: (2.0, 0.5) m/s
- Simulation time: 20 s
- Time step: 0.1 s

### Kalman Filter Parameters

- Initial covariance P: 10.0
- Process noise Q: 0.01
- Measurement noise R: 4.0
- State: [x, y]
- Stationary emitter model

### Results

- Rolling LS mean localization error: 18.019 m
- Kalman mean localization error: 20.863 m

### Observation

Rolling-window least-squares localization produced significantly larger
errors than cumulative least squares.

The current UAV trajectory is almost linear, and measurements inside a
short rolling window have similar bearing geometry. This makes emitter
triangulation poorly conditioned.

The Kalman filter cannot fully compensate for biased or geometrically
ill-conditioned position measurements.

### Next Experiment

Improve UAV measurement geometry before further Kalman filter tuning.

Use a curved UAV trajectory so that AOA measurements are collected from
more diverse viewing angles.

## Experiment 03 - Curved UAV Path + Rolling LS + Kalman Filter

### Setup

- AOA noise standard deviation: 2 deg
- Rolling window size: 30
- Emitter position: (30, 30) m
- UAV start position: (0, 5) m
- UAV trajectory: curved path
- X velocity: 2.0 m/s
- Curve rate: 0.04
- Simulation time: 20 s
- Time step: 0.1 s

### Kalman Filter Parameters

- Initial covariance P: 10.0
- Process noise Q: 0.01
- Measurement noise R: 4.0
- State: [x, y]
- Stationary emitter model

### Results

- Final LS localization error: 0.267 m
- Rolling LS mean localization error: 7.517 m
- Kalman mean localization error: 9.475 m

### Observation

Changing the UAV trajectory from a nearly straight path to a curved path
significantly improved AOA localization performance.

The rolling LS mean error decreased from 18.019 m in Experiment 02
to 7.517 m.

This indicates that measurement geometry strongly affects AOA-based
localization accuracy.

However, the Kalman-filtered estimate still produced a larger mean error
than the raw rolling LS estimate.

### Next Experiment

Analyze Kalman filter covariance parameters and measurement characteristics.

Evaluate the effects of P, Q, and R before moving to Extended Kalman Filter
implementation.

## Experiment 04 - Kalman Filter R Sensitivity

### Fixed Parameters

- Process noise Q: 0.01
- Initial covariance P: 10.0
- Rolling window size: 30
- Curved UAV trajectory

### Results

- Rolling LS mean error: 7.517 m

| R | Kalman Mean Error |
|---:|---:|
| 0.5 | 8.236 m |
| 1.0 | 8.540 m |
| 2.0 | 8.946 m |
| 4.0 | 9.475 m |
| 8.0 | 10.129 m |
| 16.0 | 10.874 m |

### Observation

Kalman filter performance improved as measurement noise R decreased.

A smaller R causes the filter to place more weight on the rolling
least-squares measurements.

However, even the best tested value, R = 0.5, produced a mean error
of 8.236 m, which was higher than the raw rolling LS error of 7.517 m.

This suggests that the current stationary linear Kalman model does not
provide additional predictive information beyond the LS position estimate.

### Conclusion

Further parameter tuning alone is unlikely to solve the fundamental
limitation of this filtering structure.

The next step is to use the original nonlinear AOA measurement directly
with an Extended Kalman Filter.

## Experiment 05 - Extended Kalman Filter

### Setup

- AOA noise standard deviation: 2 deg
- Curved UAV trajectory
- Initial EKF state generated from first 30 AOA measurements
- State: [emitter_x, emitter_y]
- Nonlinear AOA measurement model
- Initial covariance P: 100.0
- Process noise Q: 0.01

### Results

- Final LS localization error: 0.267 m
- Rolling LS mean error: 7.517 m
- Linear KF mean error: 8.236 m
- EKF mean error: 2.502 m

- Initial EKF state:
  [15.7823, 16.7795]

- Final EKF state:
  [29.8700, 30.2655]

### Observation

The Extended Kalman Filter significantly outperformed both rolling
least-squares localization and the linear Kalman Filter in mean
localization error.

The EKF directly processed nonlinear AOA measurements using the
atan2 measurement model and its Jacobian.

Despite a poor initial position estimate, the EKF converged close to
the true emitter position.

### Conclusion

Direct nonlinear AOA measurement modeling was more effective than
applying a linear Kalman Filter to preprocessed least-squares position
estimates.