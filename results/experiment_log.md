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


## Experiment 9 — ROS2 Multi-Node Localization Integration

### Objective

Extend the existing standalone AOA localization simulation into a ROS2-based distributed architecture.

### ROS2 Architecture

The original localization pipeline was separated into independent ROS2 nodes.

#### UAV Position Publisher

- Node: `uav_position_publisher`
- Publishes: `/uav_position`

#### AOA Measurement Node

- Node: `aoa_measurement_node`
- Subscribes: `/uav_position`
- Publishes: `/aoa_measurement`

#### EKF Localization Node

- Node: `ekf_localization_node`
- Uses UAV position and AOA measurements
- Publishes: `/emitter_estimate`

### Result

The UAV position, AOA measurement, and EKF localization functions were successfully separated into independent ROS2 nodes.

The following topics were verified:

- `/uav_position`
- `/aoa_measurement`
- `/emitter_estimate`
- `/rosout`

The experiment confirmed that the original standalone localization algorithm could operate through ROS2 topic-based communication.

### Conclusion

The standalone localization pipeline was successfully converted into a ROS2 multi-node architecture.

This provided the communication structure required for subsequent trajectory experiments and closed-loop active localization.


## Experiment 10 — Trajectory Geometry and EKF Localization

### Objective

Evaluate how UAV trajectory geometry affects AOA-based EKF localization performance.

### Straight Trajectory

The initial ROS2 experiment used an approximately straight UAV trajectory.

Although the EKF continuously received AOA measurements, the emitter estimate converged to a biased position around:

`(26, 29)` m

while the true emitter position was:

`(30, 30)` m.

This indicated that simply increasing the number of AOA measurements was not sufficient for accurate localization.

Successive measurements collected along a nearly straight trajectory provided similar observation geometry, limiting the geometric diversity available to the estimator.

### Curved Trajectory

The UAV trajectory was then modified to continuously change the observation geometry relative to the emitter.

Example result:

```text
UAV=(98.50, 19.99) EKF=(30.313, 29.939)
UAV=(100.00, 19.84) EKF=(30.372, 29.733)
UAV=(103.50, 18.66) EKF=(30.373, 29.742)
UAV=(107.50, 16.02) EKF=(30.388, 29.461)
```

The EKF estimate moved substantially closer to the true emitter position compared with the straight-trajectory experiment.

### Result

Changing the UAV trajectory improved the diversity of AOA observation angles and resulted in more accurate emitter localization.

### Conclusion

The experiment demonstrated that AOA localization performance depends not only on the estimator but also on the geometry of the measurements collected by the moving sensor.

This result motivated the implementation of an active trajectory planner that selects UAV observation positions based on their expected information contribution.


## Experiment 11 — FIM-Based Active Localization

### Objective

Replace the predefined UAV trajectory with an active waypoint planner based on the Fisher Information Matrix (FIM).

### Method

For each candidate UAV position, the expected information contribution of an AOA measurement was calculated using:

`FIM = H^T R^-1 H`

where:

- `H` is the AOA measurement Jacobian.
- `R` is the measurement noise covariance.

Sixteen candidate positions were generated around the current UAV position.

For each candidate, the predicted information matrix was evaluated using the log determinant of the FIM.

A candidate with a larger log determinant was considered to provide more informative observation geometry for emitter localization.

The selected candidate was published through:

`/next_waypoint`

The localization pipeline was extended to:

```text
UAV Position
    ↓
AOA Measurement
    ↓
EKF Localization
    ↓
FIM Planner
    ↓
Next Waypoint
    ↓
UAV Motion
```

### Result

The FIM planner successfully generated waypoints using the current UAV position and EKF emitter estimate.

Instead of following a predefined trajectory, the UAV could select its next observation position according to the expected information contribution of each candidate.

### Conclusion

The localization system was extended from passive estimation with a predefined trajectory to closed-loop active sensing.

The current emitter estimate could now influence the UAV's next observation position, providing the basis for information-driven trajectory planning.


## Experiment 12 — Closed-Loop Waypoint Control

### Objective

Stabilize UAV waypoint following in the closed-loop FIM planning system.

### Initial Problem

During the initial closed-loop experiment, the FIM planner continuously generated new waypoints while the UAV was still moving toward the previous waypoint.

As a result, the target direction changed frequently and the UAV showed unstable and repetitive motion.

The planner and UAV controller were operating correctly as individual components, but the planner was updating the target faster than the UAV could reach it.

### Arrival-Based Replanning

The waypoint control strategy was modified to:

```text
Plan
→ Move
→ Reach Waypoint
→ Re-plan
```

After a waypoint was selected, the planner waited until the UAV reached the current target before generating the next waypoint.

This prevented continuous waypoint replacement during UAV movement.

### Immediate Direction Reversal

After arrival-based replanning was introduced, another repetitive behavior was observed:

```text
A → B → A → B
```

Although the UAV now completed each waypoint command, the FIM criterion could select a new waypoint in the direction opposite to the previous movement.

To reduce this behavior, a reversal penalty was added to the candidate evaluation.

The planner score was modified to:

`score = information score - reversal penalty`

The movement direction of each candidate was compared with the previous movement direction using a dot product.

Candidates representing strong backward motion received a larger penalty.

### Result

Arrival-based replanning eliminated continuous target changes while the UAV was moving.

The reversal penalty also reduced immediate back-and-forth motion between consecutive waypoints.

### Conclusion

The experiment showed that maximizing measurement information alone was not sufficient to generate a practical UAV trajectory.

Additional motion constraints were required to convert FIM-based waypoint selection into stable closed-loop UAV movement.

However, longer repetitive trajectories involving previously visited regions were still observed, motivating the addition of a revisit penalty in the next experiment.


## Experiment 13 — Revisit Suppression and FIM Trajectory Improvement

### Objective

Reduce longer repetitive trajectories that were not prevented by the immediate reversal penalty.

### Problem

Although the reversal penalty reduced immediate back-and-forth motion, the UAV could still return to recently visited regions after several waypoint transitions.

For example:

```text
A → B → C → D → A
```

This behavior could not be prevented by the reversal penalty alone because returning to a previously visited location does not necessarily correspond to an immediate direction reversal.

### Revisit Penalty

A history of recently visited waypoints was added to the FIM planner.

Each candidate position was compared with the recent waypoint history.

Candidates located near recently visited positions received an additional penalty.

The candidate evaluation score was modified to:

`score = information score - reversal penalty - revisit penalty`

The planner parameters used in the experiment were:

- Candidate step size: `5 m`
- Number of candidate directions: `16`
- Reversal penalty weight: `1.5`
- Revisit penalty weight: `2.0`
- Revisit radius: `3 m`
- Recent waypoint history: `8`
- Minimum emitter distance: `3 m`

### Minimum Emitter-Distance Constraint

A minimum-distance constraint was also introduced.

Candidate positions located too close to the current emitter estimate were rejected.

This prevented the planner from repeatedly selecting observation positions directly around the estimated emitter location.

### Final Trajectory

Example regions visited during the improved closed-loop trajectory included:

```text
(38.7, 25.8)
→ (34.1, 23.9)
→ (29.5, 25.8)
→ (26.0, 29.3)
→ (27.9, 33.9)
→ (23.3, 35.8)
→ (21.3, 31.2)
→ (23.3, 26.6)
→ (26.8, 23.1)
→ (31.4, 21.2)
→ (33.3, 25.8)
→ (33.3, 30.8)
→ (31.4, 35.4)
```

Compared with the earlier repetitive trajectories, the UAV collected observations from a wider range of directions around the emitter region.

### Result

The combination of the reversal penalty and revisit penalty reduced short repetitive waypoint patterns.

The minimum emitter-distance constraint also prevented candidate selection from collapsing directly toward the estimated emitter position.

### Conclusion

FIM-based information maximization was successfully combined with simple trajectory constraints to generate a more diverse observation path.

The resulting planner preserved information-driven waypoint selection while reducing undesirable repetitive motion.


## Experiment 14 — Final ROS2 Closed-Loop Verification

### Objective

Verify the complete ROS2-based active emitter localization system after integrating the EKF estimator, FIM planner, and UAV waypoint controller.

### Final ROS2 Architecture

The complete system consisted of four ROS2 nodes:

- `uav_position_publisher`
- `aoa_measurement_node`
- `ekf_localization_node`
- `fim_planner_node`

The main communication topics were:

- `/uav_position`
- `/aoa_measurement`
- `/emitter_estimate`
- `/next_waypoint`

All four nodes were integrated into a single ROS2 launch file and executed using:

```powershell
ros2 launch uav_localization_ros localization.launch.py
```

### Localization Result

During the closed-loop experiment, the EKF estimate converged close to the true emitter position.

One observed result was:

```text
True emitter = (30.000, 30.000)
EKF estimate = (29.978, 29.976)
```

The corresponding Euclidean localization error was approximately:

`0.033 m`

This value represents one observed simulation result rather than an averaged statistical performance measurement.

### ROS2 Runtime Debugging

During final testing, discontinuous UAV position values were observed on `/uav_position`.

The ROS2 graph was inspected using:

```powershell
ros2 topic info /uav_position -v
```

The inspection showed:

```text
Publisher count: 2
```

Two ROS2 launch instances were simultaneously publishing UAV positions to the same topic.

This caused position messages from two independent UAV trajectories to appear alternately on `/uav_position`.

After terminating the duplicated ROS2 processes and restarting a single launch instance, the topic configuration was verified again:

```text
Publisher count: 1
Subscription count: 2
```

The two subscribers were:

- `aoa_measurement_node`
- `fim_planner_node`

The UAV trajectory then returned to continuous waypoint-following behavior.

### Final Trajectory Verification

With a single ROS2 launch instance, the UAV followed the FIM-selected waypoints continuously.

The final trajectory showed movement through multiple observation directions around the emitter region rather than the previously observed short repetitive patterns.

The approximately `1 m` position change between consecutive updates was also consistent with the configured UAV speed and update interval.

### Result

The complete closed-loop pipeline operated successfully:

```text
UAV Position
    ↓
AOA Measurement
    ↓
EKF Localization
    ↓
FIM-Based Waypoint Planning
    ↓
UAV Waypoint Following
    ↓
New Observation Geometry
    ↓
AOA Measurement
```

The final system successfully integrated:

- ROS2 multi-node communication
- Simulated AOA measurements
- EKF emitter localization
- FIM-based waypoint selection
- Waypoint-following UAV motion
- Arrival-based replanning
- Direction reversal suppression
- Recent-position revisit suppression
- Minimum emitter-distance constraint
- ROS2 launch integration
- Closed-loop active localization

### Conclusion

The standalone emitter localization simulation was successfully extended into a ROS2-based closed-loop active localization system.

The experiments demonstrated that localization performance depends not only on the estimation algorithm but also on the observation geometry generated by the UAV trajectory.

By using the current emitter estimate to select informative future observation positions, the system progressed from passive localization with a predefined trajectory to active sensing with feedback-based trajectory planning.

The final ROS2 integration also demonstrated that runtime communication issues, such as duplicated publishers, can be distinguished from localization algorithm problems through ROS2 graph and topic inspection.