# Multi-Car Mode - Documentation

## Overview

Multi-car mode enables head-to-head racing with 2-4 autonomous vehicles on the same track. This mode supports:

- **Different planners per car** for algorithm comparison
- **Performance scaling** to create speed differentials
- **Collision detection** and safety systems
- **Synchronized race start** via services

---

## Launch Command

### Basic Usage

```bash
# Default: 2 cars with spliner planner on map M1
roslaunch stack_master multi_car.launch

# Custom map for all cars
roslaunch stack_master multi_car.launch global_map:=f_100%s_100%w_NUC2_mintime
```

### Different Planners Per Car

```bash
# Car1: Predictive Sampler (hybrid), Car2: Predictive Spliner
roslaunch stack_master multi_car.launch \
    planner_car1:=predictive_sampler \
    planner_car2:=predictive_spliner \
    global_map:=f_100%s_100%w_NUC2_mintime

# Car1: TAM Sampling, Car2: Spliner
roslaunch stack_master multi_car.launch \
    planner_car1:=tam_sampling \
    planner_car2:=spliner
```

### Performance Scaling

Create speed differentials between cars for overtaking scenarios:

```bash
# Car2 at 60% speed and acceleration (slower opponent)
roslaunch stack_master multi_car.launch \
    planner_car1:=predictive_sampler \
    planner_car2:=predictive_spliner \
    speed_multiplier_car2:=0.6 \
    accel_multiplier_car2:=0.6

# Both cars at different speeds
roslaunch stack_master multi_car.launch \
    speed_multiplier_car1:=1.0 \
    speed_multiplier_car2:=0.7 \
    accel_multiplier_car1:=1.0 \
    accel_multiplier_car2:=0.7
```

### Full Featured Example

```bash
roslaunch stack_master multi_car.launch \
    planner_car1:=predictive_sampler \
    planner_car2:=predictive_spliner \
    global_map:=f_100%s_100%w_NUC2_mintime \
    speed_multiplier_car2:=0.6 \
    accel_multiplier_car2:=0.6 \
    enable_race_start_controller:=true \
    use_global_prediction:=true \
    rviz:=true
```

---

## Starting the Race

```bash
# Start both cars simultaneously
rosservice call /race_control/start_both

# Start cars individually
rosservice call /race_control/start_car1
rosservice call /race_control/start_car2

# Reset to starting positions
rosservice call /race_control/reset_cars

# Emergency stop
rosservice call /race_control/emergency_stop
```

---

## Launch Arguments Reference

### Global Configuration

| Argument | Default | Description |
|----------|---------|-------------|
| `global_map` | `M1` | Map for all cars (can be overridden per car) |
| `sim` | `True` | Simulation flag for all cars |
| `rviz` | `True` | Launch consolidated RViz |
| `use_global_map` | `True` | Use global neutral map frame |
| `global_map_frame` | `map` | Global map frame name |

### Multi-Car Interaction

| Argument | Default | Description |
|----------|---------|-------------|
| `car_model` | `NUC2` | Car model type |
| `enable_car_interaction` | `True` | Enable inter-car perception/collision detection |
| `enable_collision_detection` | `True` | Enable collision detection |
| `collision_warning_distance` | `1.5` | Warning distance (meters) |
| `collision_critical_distance` | `0.8` | Critical distance (meters) |

### Race Control

| Argument | Default | Description |
|----------|---------|-------------|
| `enable_race_start_controller` | `True` | Enable manual race start control |
| `use_global_prediction` | `false` | Use global waypoint prediction (testing mode) |

### Performance Scaling (Per Car)

| Argument | Default | Description |
|----------|---------|-------------|
| `speed_multiplier_car1` | `1.0` | Speed multiplier for car1 |
| `speed_multiplier_car2` | `1.0` | Speed multiplier for car2 |
| `speed_multiplier_car3` | `1.0` | Speed multiplier for car3 |
| `speed_multiplier_car4` | `1.0` | Speed multiplier for car4 |
| `accel_multiplier_car1` | `1.0` | Acceleration multiplier for car1 |
| `accel_multiplier_car2` | `1.0` | Acceleration multiplier for car2 |
| `accel_multiplier_car3` | `1.0` | Acceleration multiplier for car3 |
| `accel_multiplier_car4` | `1.0` | Acceleration multiplier for car4 |

### Per-Car Configuration

| Argument | Default | Description |
|----------|---------|-------------|
| `car1` | `car1` | Car 1 namespace (empty = disabled) |
| `car2` | `car2` | Car 2 namespace (empty = disabled) |
| `car3` | `` | Car 3 namespace (empty = disabled) |
| `car4` | `` | Car 4 namespace (empty = disabled) |
| `planner_car1..4` | `spliner` | Planner for each car |
| `map_car1..4` | `$(arg global_map)` | Map for each car |
| `version_car1..4` | `NUC2` | Version for each car |
| `frame_prefix_car1..4` | `car1_..car4_` | TF frame prefix for each car |

### Initial Position (Per Car)

| Argument | Default | Description |
|----------|---------|-------------|
| `car1_init_x/y/theta` | Map-specific | Car 1 initial pose (at s=0) |
| `car2_init_x/y/theta` | Map-specific | Car 2 initial pose (at s=3) |
| `car3_init_x/y/theta` | `-1.0/-1.5/0.0` | Car 3 initial pose |
| `car4_init_x/y/theta` | `1.0/1.5/0.0` | Car 4 initial pose |

---

## Available Planners

| Planner | Key | Description |
|---------|-----|-------------|
| Spliner | `spliner` | Basic spline-based overtaking planner |
| Predictive Spliner | `predictive_spliner` | GP-based opponent prediction + SQP optimization |
| TAM Sampling | `tam_sampling` | Lateral/longitudinal trajectory sampling |
| Predictive Sampler | `predictive_sampler` | Hybrid: GP prediction + TAM sampling |
| Graph-Based | `graph_based` | Discrete graph search |
| Frenet | `frenet` | Polynomial trajectory optimization |

---

## Topic Namespacing

In multi-car mode, each car has its own namespace:

```
/car1/global_waypoints
/car1/car_state/odom_frenet
/car1/planner/avoidance/otwpnts
/car1/perception/obstacles

/car2/global_waypoints
/car2/car_state/odom_frenet
/car2/planner/avoidance/otwpnts
/car2/perception/obstacles
```

### Frame Prefixes

TF frames are prefixed with car namespace:
- `car1_base_link`, `car1_laser`
- `car2_base_link`, `car2_laser`

---

## Example Configurations

### Algorithm Comparison (Same Speed)

```bash
roslaunch stack_master multi_car.launch \
    planner_car1:=predictive_sampler \
    planner_car2:=tam_sampling \
    global_map:=f_100%s_100%w_NUC2_mintime

rosservice call /race_control/start_both
```

### Overtaking Scenario (Slower Opponent)

```bash
roslaunch stack_master multi_car.launch \
    planner_car1:=predictive_spliner \
    planner_car2:=spliner \
    speed_multiplier_car2:=0.5 \
    accel_multiplier_car2:=0.5

rosservice call /race_control/start_both
```

### Testing with Global Prediction

```bash
roslaunch stack_master multi_car.launch \
    planner_car1:=predictive_sampler \
    planner_car2:=predictive_spliner \
    use_global_prediction:=true

rosservice call /race_control/start_both
```

### Three Cars (Experimental)

```bash
roslaunch stack_master multi_car.launch \
    car3:=car3 \
    planner_car1:=predictive_sampler \
    planner_car2:=spliner \
    planner_car3:=tam_sampling

rosservice call /race_control/start_both
```

> **Note**: Cars 3 and 4 are experimental stubs. Full support requires additional relay and frenet republisher setup.

---

## Collision Detection

### Collision Distances

| Level | Distance | Action |
|-------|----------|--------|
| Warning | 1.5m | Logged, no action |
| Critical | 0.8m | Logged, caution |
| Collision | 0.4m | Race may end |

### Monitoring Collisions

```bash
# Check race event monitor output
rostopic echo /race_events

# Check for collisions
rosparam get /race_test/collision_count
```

---

## Monitoring & Debugging

### Key Topics

```bash
# Car 1 state
rostopic echo /car1/car_state/odom_frenet

# Car 2 planned trajectory
rostopic echo /car2/planner/avoidance/otwpnts

# Race events
rostopic echo /race_events
```

### Useful Commands

```bash
# Check current states
rosparam get /car1/state_machine/current_state
rosparam get /car2/state_machine/current_state

# Check race completion
rosparam get /race_test/simulation_complete

# Force race end
rosparam set /simulation_complete true
```

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Cars don't move | Call `rosservice call /race_control/start_both` |
| Wrong starting positions | Check `car1_init_x/y/theta`, `car2_init_x/y/theta` |
| Collision detection too sensitive | Adjust `collision_critical_distance` |
| Performance too similar | Use `speed_multiplier_carX` and `accel_multiplier_carX` |
| Cars 3/4 not working | These are experimental stubs - use cars 1/2 |
| Map mismatch | Ensure all cars use same `global_map` |

---

## Architecture Notes

### Shared Components

- **Global Map Server**: Single map server shared by all cars
- **Global Waypoint Publisher**: Common trajectory reference
- **RViz**: Single consolidated visualization

### Per-Car Components

- Simulator instance (namespaced)
- Localization (SLAM/PF2)
- State machine
- Planner
- Controller
- Perception (inter-car detection)

### Multi-Car Interaction System

The `multi_car_interaction` node enables:
- Inter-car perception (detecting other cars)
- Collision detection and logging
- Race event monitoring
