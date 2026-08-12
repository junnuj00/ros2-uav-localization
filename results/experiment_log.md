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


## Experiment 06 - FIM-based Candidate Waypoint Selection

### Setup

- Current UAV position:
  [40.0, 21.0]

- EKF emitter estimate:
  [29.8700, 30.2655]

- Number of candidate waypoints: 8
- Candidate radius: 5 m
- AOA noise standard deviation: 2 deg
- Waypoint metric: determinant of cumulative Fisher Information Matrix

### Results

Candidate FIM scores:

- Candidate 0: 26449.273
- Candidate 1: 26849.731
- Candidate 2: 27714.337
- Candidate 3: 28093.588
- Candidate 4: 26765.997
- Candidate 5: 26294.528
- Candidate 6: 26211.992
- Candidate 7: 26270.153

Best waypoint:

[36.4645, 24.5355]

Best FIM score:

28093.588

### Observation

The FIM-based waypoint evaluation selected candidate 3,
located to the upper-left of the current UAV position.

The selected waypoint maximized the determinant of the cumulative
Fisher Information Matrix, indicating that the expected AOA
measurement at this position provides the largest increase in
localization information among the tested candidates.

### Next Step

Apply candidate waypoint selection repeatedly to generate an
information-aware UAV trajectory.



## Experiment 07 - FIM-based Adaptive Path Planning

### Setup

- True emitter position: (30, 30) m
- AOA noise standard deviation: 2 deg
- Localization method: Extended Kalman Filter
- Candidate waypoints: 8
- Candidate radius: 5 m
- Adaptive planning steps: 20
- Waypoint selection metric: determinant of cumulative Fisher Information Matrix
- EKF initial covariance P: 100.0
- EKF process noise Q: 0.01

### Results

- Curved-path EKF mean error: 2.502 m

- FIM-path initial estimate:
  [33.5405, 34.5245]

- FIM-path final EKF estimate:
  [30.0028, 29.9643]

- FIM-path EKF mean error: 1.404 m
- FIM-path final localization error: 0.036 m

- Mean error reduction compared with curved-path EKF:
  approximately 43.9%

### Observation

The FIM-based adaptive path achieved a lower mean localization error
than the predefined curved UAV trajectory.

At each planning step, candidate waypoints were evaluated using the
determinant of the cumulative Fisher Information Matrix.

The waypoint with the highest FIM score was selected, a new AOA
measurement was generated at that position, and the EKF estimate was
updated.

The EKF estimate converged from approximately (33.54, 34.52) m
to (30.00, 29.96) m, close to the true emitter position.

### Conclusion

FIM-based adaptive waypoint selection reduced the EKF mean localization
error from 2.502 m to 1.404 m in this simulation.

This corresponds to an approximately 43.9% reduction in mean
localization error compared with the predefined curved trajectory.

The result demonstrates that UAV trajectory design can improve
AOA-based emitter localization by selecting measurement positions
with more informative observation geometry.

### Next Step

Calculate the Cramer-Rao Lower Bound (CRLB) from the Fisher Information
Matrix and compare the theoretical localization uncertainty of the
predefined curved path and the FIM-based adaptive path.


## Experiment 08 - Equal-Budget CRLB Comparison

### Setup

- True emitter position: (30, 30) m
- AOA noise standard deviation: 2 deg
- Measurement budget: 25
- Baseline trajectory: predefined curved UAV path
- Adaptive trajectory: FIM-based UAV path
- CRLB metric: sqrt(trace(inv(FIM)))
- Equal number of measurement positions used for both trajectories

### Results

- Measurement budget: 25

- Curved-path CRLB position bound:
  0.3379 m

- FIM-path CRLB position bound:
  0.0553 m

- CRLB reduction:
  83.62%

### Observation

The FIM-based trajectory achieved a substantially lower CRLB position
bound than the predefined curved trajectory under the same measurement
budget.

The curved trajectory used 25 uniformly sampled measurement positions
from the original trajectory, while the FIM-based trajectory also
contained 25 measurement positions.

Because both trajectories were evaluated using the same number of
measurements, the difference in CRLB primarily reflects the effect of
measurement geometry rather than the number of observations.

### Conclusion

The FIM-based adaptive trajectory reduced the theoretical position
uncertainty bound from 0.3379 m to 0.0553 m.

This corresponds to an 83.62% reduction in the CRLB position bound
compared with the predefined curved trajectory under the same
measurement budget.

Combined with the EKF experiment, the results indicate that
information-aware UAV path planning can improve both theoretical
localization observability and simulated emitter localization
performance.

### Next Step

Integrate the completed AOA measurement, EKF localization, and
FIM-based path planning modules into a ROS2 node and topic architecture.