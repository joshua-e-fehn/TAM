# TAM Custom Predictor - Constant Offset Prediction

This package implements a custom constant offset predictor for the TAM sampling planner, specifically designed for multi-car racing scenarios.

## Overview

The TAM Custom Predictor provides a simple but effective prediction method for opponent vehicles based on the principle that **opponents maintain their current lateral offset to the raceline**. 

### Prediction Logic

- **Current State**: If an opponent is driving 2 meters left of the raceline
- **Future Prediction**: The opponent will continue driving 2 meters left of the raceline for the entire prediction horizon
- **Boundary Handling**: Predictions are constrained to stay within track boundaries with safety margins
- **Smoothing**: Boundary transitions are smoothed to avoid abrupt trajectory changes

## Architecture

### Nodes

1. **`tam_prediction_node.py`**: Main prediction node
   - Subscribes to obstacle data and global waypoints
   - Generates constant offset predictions
   - Publishes predictions and visualization markers

2. **TAM Sampling Planner Integration**: Enhanced planner node
   - Subscribes to prediction data
   - Integrates predictions into trajectory planning
   - Considers predicted opponent positions for collision avoidance

### Topics

#### Subscribed Topics
- `perception/obstacles` (`ObstacleArray`): Current obstacle positions
- `global_waypoints` (`WpntArray`): Track centerline for Frenet conversion  
- `car_state/odom_frenet` (`Odometry`): Ego vehicle state (optional, for horizon calculation)

#### Published Topics
- `prediction/waypoints` (`OpponentTrajectory`): Predicted opponent trajectories
- `prediction/markers` (`MarkerArray`): Visualization markers for RViz

## Parameters

### Core Prediction Parameters
- `prediction_horizon` (default: 5.0): Prediction time horizon in seconds
- `prediction_dt` (default: 0.1): Time step between prediction points
- `prediction_points`: Calculated as `horizon / dt`

### Safety Parameters  
- `safety_margin` (default: 0.3): Distance from track boundaries in meters
- `smoothing_factor` (default: 0.8): Smoothing factor for boundary transitions

## Usage

### Basic Usage

Launch the complete TAM system with custom prediction:

```bash
# Single car with TAM prediction
roslaunch tam_sampling_planner tam_with_prediction.launch car_namespace:=car1

# Multi-car setup
roslaunch tam_sampling_planner tam_with_prediction.launch car_namespace:=car1 &
roslaunch tam_sampling_planner tam_with_prediction.launch car_namespace:=car2 &
```

### Test Prediction Only

Test the predictor node independently:

```bash
# Test predictor without full planner
roslaunch tam_sampling_planner tam_predictor_test.launch car_namespace:=car1

# Adjust prediction parameters
roslaunch tam_sampling_planner tam_predictor_test.launch \
    car_namespace:=car1 \
    prediction_horizon:=10.0
```

### Integration with Existing Systems

The predictor integrates seamlessly with existing F1TENTH racing stacks:

```bash
# Use with existing multi-car launch system
roslaunch stack_master multi_car.launch \
    cars:=car1,car2 \
    planners:=tam_sampling,tam_sampling
```

## Algorithm Details

### 1. Obstacle Detection & Filtering
- Filters dynamic obstacles from `perception/obstacles`
- Converts obstacle positions to Frenet coordinates using track centerline
- Calculates current lateral offset (d) and longitudinal position (s)

### 2. Constant Offset Prediction
- **Assumption**: Lateral offset `d` remains constant over time
- **Longitudinal Motion**: Progresses with estimated velocity along track
- **Time Horizon**: Generates predictions up to specified horizon

```python
# Prediction logic (simplified)
for t in prediction_times:
    s_future = (s_current + velocity * t) % track_length
    d_future = d_current  # Constant offset assumption
    
    # Apply boundary constraints
    d_future = constrain_to_track_boundaries(s_future, d_future)
```

### 3. Boundary Constraint Handling
- Ensures predictions stay within track boundaries
- Applies safety margins to prevent track limit violations
- Smooth transitions when hitting boundaries to avoid abrupt changes

### 4. Message Generation
- Converts predictions to `OpponentTrajectory` format
- Creates visualization markers for RViz display
- Publishes continuously for real-time planning

## Integration with TAM Sampling Planner

### Enhanced Obstacle Processing
The TAM sampling planner now includes prediction integration:

```python
def process_obstacles(self):
    """Process both current obstacles and predictions"""
    obstacles = process_current_obstacles(self.obs)
    
    if self.predictions.oppwpnts:
        predicted_obstacles = process_predictions(self.predictions)
        obstacles.extend(predicted_obstacles)
    
    return obstacles
```

### Prediction-Aware Planning
- Predictions are treated as future obstacles in trajectory sampling
- Cost function includes prediction-based collision avoidance
- Planning considers opponent behavior over the prediction horizon

## Advantages & Limitations

### Advantages ✅
- **Simple & Fast**: Computationally efficient for real-time racing
- **Realistic for Racing**: Many racing scenarios involve consistent driving lines
- **Track Boundary Aware**: Respects physical constraints of the track
- **Smooth Predictions**: Avoids erratic predicted behavior

### Limitations ⚠️
- **Assumes Constant Behavior**: Cannot predict lane changes or strategic moves
- **No Velocity Changes**: Assumes constant longitudinal velocity
- **Limited Interaction Modeling**: Doesn't model car-to-car interactions

## Performance Characteristics

- **Update Rate**: Up to 10 Hz (triggered by obstacle updates)
- **Prediction Points**: Typically 50 points (5s @ 0.1s resolution)
- **Computational Cost**: Very low - linear in number of obstacles
- **Memory Usage**: Minimal - stores only current state and recent predictions

## Future Enhancements

Potential improvements for more sophisticated prediction:

1. **Velocity Profile Prediction**: Consider acceleration/deceleration patterns
2. **Multi-Modal Predictions**: Generate multiple trajectory hypotheses  
3. **Interaction-Aware**: Model responses to ego vehicle behavior
4. **Learning-Based**: Use historical data to improve predictions
5. **Strategic Prediction**: Incorporate racing strategy and overtaking models

## Troubleshooting

### Common Issues

**No predictions generated:**
- Check if `perception/obstacles` contains dynamic obstacles
- Verify `global_waypoints` are being published
- Ensure Frenet converter initialization succeeds

**Predictions hitting track boundaries:**
- Increase `safety_margin` parameter
- Check track boundary data in global waypoints
- Verify `d_left` and `d_right` fields are correct

**Performance issues:**
- Reduce `prediction_horizon` or increase `prediction_dt`
- Check for excessive obstacle count
- Monitor computational load with `top` or `htop`

### Debug Topics

Monitor these topics for debugging:

```bash
# Check obstacle input
rostopic echo /car1/perception/obstacles

# Monitor predictions output  
rostopic echo /car1/prediction/waypoints

# Visualize in RViz
rostopic echo /car1/prediction/markers
```

## Dependencies

- ROS1 (tested with Melodic/Noetic)
- Python 3.6+
- NumPy, SciPy
- Custom packages: `f110_msgs`, `frenet_converter`
- TAM sampling planner core

## References

- TAM Racing Team: [TUM Autonomous Motorsport](https://www.mw.tum.de/en/ftm/research/vehicle-dynamics-and-control-systems/current-projects/autonomous-motorsport/)
- F1TENTH: [F1TENTH Autonomous Racing](https://f1tenth.org/)
- Original TAM ROS2 Implementation: `tam_race_stack/mod_planning/sampling_planner/`
