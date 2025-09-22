# TAM Sampling Planner - ROS1 Integration

This package provides a ROS1 port of the TAM (TUM Autonomous Motorsport) Sampling Planner for integration with the existing multi-car racing simulation environment.

## Overview

The TAM Sampling Planner is a sophisticated trajectory planning algorithm that uses sampling-based methods in Frenet coordinates to generate optimal racing trajectories. This implementation adapts the core TAM algorithms to work seamlessly with the existing ROS1 multi-car racing architecture.

## Key Features

- **Sampling-based Planning**: Generates multiple trajectory candidates and selects the optimal one
- **Frenet Coordinates**: Plans in road-relative coordinate system for intuitive racing maneuvers  
- **Multi-objective Optimization**: Balances speed, safety, smoothness, and raceline following
- **Real-time Performance**: Designed for 20Hz planning frequency
- **Dynamic Reconfigure**: Real-time parameter tuning during operation
- **Multi-car Compatible**: Integrates with existing obstacle avoidance and multi-car systems

## Architecture

### Core Components

1. **`tam_sampling_core.py`**: Core sampling algorithms adapted from TAM ROS2 implementation
   - `TAMSamplingCore`: Main planning class with trajectory sampling and evaluation
   - `FrenetTrajectory`: Trajectory representation in Frenet coordinates
   - `TAMSamplingUtils`: Coordinate transformation utilities

2. **`tam_sampling_node.py`**: ROS1 node following existing planner patterns
   - Subscribes to global waypoints, vehicle state, and obstacles
   - Publishes optimized trajectories for the state machine
   - Handles dynamic reconfigure parameter updates

3. **`dynamic_tam_sampling_server.py`**: Dynamic reconfigure server for real-time tuning

### Integration Pattern

The TAM Sampling Planner follows the same integration pattern as the existing `spliner` and `predictive_spliner`:

```
Input Topics:
├── global_waypoints (WpntArray)          # Racing line reference
├── global_waypoints_scaled (WpntArray)   # Speed-scaled reference  
├── car_state/odom_frenet (Odometry)      # Vehicle state in Frenet coordinates
└── perception/obstacles (ObstacleArray)  # Detected obstacles

Output Topics:
├── planner/avoidance/otwpnts (OTWpntArray)  # Optimized trajectory
├── planner/avoidance/markers (MarkerArray)  # Visualization
└── planner/avoidance/latency (Float32)      # Performance metrics
```

## Usage

### Basic Usage

The TAM Sampling Planner can be selected as a planner option in the multi-car launch system:

```bash
# Single car with TAM sampling planner
roslaunch stack_master multi_car.launch cars:=car1 planners:=tam_sampling

# Multi-car with mixed planners
roslaunch stack_master multi_car.launch cars:=car1,car2 planners:=tam_sampling,spliner
```

### Parameter Tuning

The planner supports real-time parameter tuning through dynamic reconfigure:

```bash
# Launch dynamic reconfigure GUI
rosrun rqt_reconfigure rqt_reconfigure
```

Key parameters:
- **Sampling**: `lateral_samples`, `longitudinal_samples`, `planning_horizon`
- **Vehicle Limits**: `max_speed`, `max_accel`, `max_lateral_accel`
- **Cost Weights**: `raceline_cost_weight`, `velocity_cost_weight`, `obstacle_cost_weight`
- **Safety**: `safety_margin_static`, `safety_margin_dynamic`

### Configuration

Default parameters are defined in `config/tam_sampling_params.yaml`. Create custom configuration files for different racing scenarios:

```yaml
# Example: Conservative racing configuration
raceline_cost_weight: 5.0      # Higher weight = stick closer to raceline
velocity_cost_weight: 2.0      # Lower weight = more conservative speeds
safety_margin_static: 0.8      # Larger safety margins
max_speed: 15.0                 # Speed limit
```

## Algorithm Details

### Sampling Strategy

The planner generates trajectory candidates using:

1. **Lateral Sampling**: Quintic polynomials across track width
   - Targets different lateral positions relative to raceline
   - Smooth trajectory generation with boundary condition control

2. **Longitudinal Sampling**: Velocity profile variations
   - Multiple velocity targets (conservative to aggressive)
   - Acceleration-limited velocity planning

3. **Combination**: Cartesian product of lateral and longitudinal samples

### Cost Function

Multi-objective cost function inspired by TAM methodology:

- **Raceline Deviation**: Minimize distance from optimal racing line
- **Velocity Optimization**: Encourage higher speeds when safe
- **Smoothness**: Penalize high jerk for passenger comfort
- **Obstacle Avoidance**: Heavy penalties for collision risks
- **Safety Margins**: Enforce minimum distances to track boundaries

### Validation

Trajectory candidates are validated against:
- Vehicle dynamics constraints (acceleration, velocity limits)
- Track boundaries and safety margins
- Collision detection with static and dynamic obstacles
- Kinematic feasibility

## Integration Notes

### Coordinate Systems

- **Input**: Frenet coordinates (s, n) relative to track centerline
- **Planning**: Native Frenet coordinate planning
- **Output**: Converted back to Cartesian coordinates for controller

### Message Compatibility

Maintains full compatibility with existing message types:
- Input: Same as `spliner` and `predictive_spliner`
- Output: Standard `OTWpntArray` format for state machine integration

### Performance

- **Planning Frequency**: 20 Hz (matching existing planners)
- **Computation Time**: Target <50ms per planning cycle
- **Memory Usage**: Optimized for real-time operation

## Dependencies

- ROS1 (tested with Melodic/Noetic)
- Python 3.6+
- NumPy, SciPy
- Custom packages: `f110_msgs`, `state_machine`

## Troubleshooting

### Common Issues

1. **No valid trajectory found**: 
   - Check if track boundaries are too restrictive
   - Reduce `lateral_samples` or increase `safety_margin_static`
   - Verify vehicle constraint parameters

2. **Planning frequency too low**:
   - Reduce `lateral_samples` and `longitudinal_samples`
   - Decrease `planning_horizon`
   - Check system computational load

3. **Trajectory too conservative**:
   - Increase `velocity_cost_weight`
   - Decrease `raceline_cost_weight`
   - Reduce safety margins

### Debugging

Enable debug output:
```bash
roslaunch tam_sampling_planner tam_sampling_planner.launch --screen
```

Monitor performance:
```bash
rostopic echo /car1/planner/avoidance/latency
```

## Future Enhancements

- [ ] Advanced obstacle prediction integration
- [ ] Multi-car interaction awareness
- [ ] Adaptive parameter tuning based on racing conditions
- [ ] Integration with TAM's vehicle dynamics models
- [ ] Warm-starting for improved convergence

## References

- TAM ROS2 Sampling Planner: `tam_race_stack/mod_planning/sampling_planner/`
- Original TAM Publications: [TAM Racing Team Research](https://www.mw.tum.de/en/ftm/research/vehicle-dynamics-and-control-systems/current-projects/autonomous-motorsport/)
