# Single Car Mode - Documentation

## Overview

Single car mode runs a single F1Tenth vehicle on the track, with optional dummy obstacle for testing overtaking maneuvers. This mode is ideal for:

- **Time Trials**: Testing lap times without obstacles
- **Obstacle Avoidance**: Testing overtaking with configurable dummy obstacles
- **Planner Development**: Iterating on planning algorithms

---

## Launch Command

### Basic Usage

```bash
# Default: spliner planner, no obstacles, map 'f'
roslaunch stack_master single_car.launch

# With specific planner
roslaunch stack_master single_car.launch planner:=predictive_spliner

# With RViz disabled (headless)
roslaunch stack_master single_car.launch rviz:=false
```

### With Dummy Obstacle

```bash
# Enable obstacle at 50% speed
roslaunch stack_master single_car.launch \
    enable_dummy_obstacle:=true \
    obstacle_speed:=0.5

# Predictive spliner with slow obstacle
roslaunch stack_master single_car.launch \
    planner:=predictive_spliner \
    enable_dummy_obstacle:=true \
    obstacle_speed:=0.3

# TAM sampling planner with custom obstacle trajectory
roslaunch stack_master single_car.launch \
    planner:=tam_sampling \
    enable_dummy_obstacle:=true \
    obstacle_trajectory:=min_curv \
    obstacle_speed:=0.4
```

### Custom Map

```bash
# Use a specific map
roslaunch stack_master single_car.launch \
    map_name:=f_100%s_100%w_NUC2_mintime \
    planner:=predictive_spliner
```

---

## Starting the Race

The race start controller keeps the car in READY state until manually started:

```bash
# Start the race (car + obstacle if enabled)
rosservice call /race_control/start_both

# Reset cars to starting positions
rosservice call /race_control/reset_cars

# Emergency stop
rosservice call /race_control/emergency_stop
```

---

## Launch Arguments Reference

### Base System Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `sim` | `True` | Simulator (True) or physical car (False) |
| `racecar_version` | `NUC2` | Physical racecar version: `NUC2`/`JET1` |
| `map_name` | `f` | Map to load from `stack_master/maps/` |
| `tire_model` | `pacejka` | Simulator tire model: `pacejka`/`linear` |
| `algo` | `slam` | Localization algorithm: `slam`/`pf2` |
| `pf_covariance` | `True` | Propagate covariance in pf2 localization |
| `scanalign` | `False` | Launch scan-align accuracy node |
| `rviz` | `True` | Launch RViz visualization |

### Initial Position Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `car_init_x` | Map-specific | Initial X position (meters) |
| `car_init_y` | Map-specific | Initial Y position (meters) |
| `car_init_theta` | Map-specific | Initial heading angle (radians) |

> **Note**: Initial positions are automatically set based on `map_name`. Custom values can be provided to override.

### Planner & Control Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `planner` | `spliner` | Planner to use (see Available Planners) |
| `MAP_mode` | `safe` | Controller mode: `safe`/`aggressive` |
| `LU_table` | `NUC2_pacejka` | Lookup table for controller |
| `ctrl_algo` | `MAP` | Control algorithm: `MAP`/`PP`/`STMPC`/`KMPC` |
| `perception` | `False` | Enable perception module |
| `measure` | `False` | Enable performance measurements |

### Dummy Obstacle Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `enable_dummy_obstacle` | `False` | Launch dummy obstacle publisher |
| `obstacle_trajectory` | `min_curv` | Trajectory: `centerline`/`min_curv`/`shortest_path`/`min_time` |
| `obstacle_start_s` | `0` | Obstacle start position (s coordinate in meters) |
| `obstacle_speed` | `0.5` | Speed scaler (0.0-1.0, where 1.0 = 100% of max speed) |
| `obstacle_constant_speed` | `false` | Use constant speed instead of trajectory-defined profile |
| `obstacle_path_amplitude` | `0.1` | Sinusoidal lateral deviation amplitude (meters) |
| `obstacle_path_frequency` | `0.15` | Sinusoidal lateral deviation frequency (rad/m) |
| `obstacle_path_phase` | `0.0` | Sinusoidal lateral deviation phase offset (radians) |
| `obstacle_speed_amplitude` | `0.0` | Sinusoidal speed variation amplitude (m/s) |
| `obstacle_max_speed_limit` | `10.0` | Physical speed limit cap (m/s) |
| `obstacle_max_accel` | `3.0` | Maximum acceleration limit (m/s²) |

### GP Prediction Settings

| Argument | Default | Description |
|----------|---------|-------------|
| `use_global_prediction` | `false` | Use global waypoint prediction instead of GP-based trajectory prediction |

### Race Control

| Argument | Default | Description |
|----------|---------|-------------|
| `enable_race_start_controller` | `True` | Enable race start controller (state machine starts in READY) |

---

## Available Planners

| Planner | Key | Description |
|---------|-----|-------------|
| Spliner | `spliner` | Basic spline-based overtaking planner |
| Predictive Spliner | `predictive_spliner` | Uses GP for opponent trajectory prediction + SQP optimization |
| TAM Sampling | `tam_sampling` | Lateral/longitudinal trajectory sampling with cost optimization |
| Predictive Sampler | `predictive_sampler` | Hybrid: GP prediction + TAM sampling trajectory generation |
| Graph-Based | `graph_based` | Discrete graph search for trajectory planning |
| Frenet | `frenet` | Polynomial trajectory optimization in Frenet coordinates |

---

## Example Configurations

### Time Trial (Fastest Lap)

```bash
roslaunch stack_master single_car.launch \
    planner:=spliner \
    map_name:=f_100%s_100%w_NUC2_mintime \
    rviz:=true

rosservice call /race_control/start_both
```

### Overtaking Test with Slow Obstacle

```bash
roslaunch stack_master single_car.launch \
    planner:=predictive_spliner \
    enable_dummy_obstacle:=true \
    obstacle_speed:=0.3 \
    obstacle_trajectory:=min_curv

rosservice call /race_control/start_both
```

### Predictive Sampler with Weaving Obstacle

```bash
roslaunch stack_master single_car.launch \
    planner:=predictive_sampler \
    enable_dummy_obstacle:=true \
    obstacle_speed:=0.4 \
    obstacle_path_amplitude:=0.2 \
    obstacle_path_frequency:=0.1

rosservice call /race_control/start_both
```

### GP Analysis with Data Saving

```bash
# First, enable GP data saving
rosparam set /race_test/save_gp_data true
rosparam set /race_test/gp_data_save_path /home/atlas/catkin_ws/gp_tests/gp_data

# Launch with predictive planner
roslaunch stack_master single_car.launch \
    planner:=predictive_spliner \
    enable_dummy_obstacle:=true \
    obstacle_speed:=0.4

rosservice call /race_control/start_both
```

---

## Important Notes

### Perception Auto-Disable

When `enable_dummy_obstacle:=true`, the perception module is **automatically disabled** to prevent topic conflicts. Both the dummy obstacle publisher and perception module publish to `/perception/obstacles`.

### Race Start Controller

When `enable_race_start_controller:=True` (default):
1. The state machine starts in **READY** state
2. Car(s) will not move until start service is called
3. Use `rosservice call /race_control/start_both` to begin

To disable (for immediate start on launch):
```bash
roslaunch stack_master single_car.launch enable_race_start_controller:=false
```

### Topic Namespacing

In single-car mode, topics are published to the global namespace (no prefix):
- `/global_waypoints`
- `/car_state/odom_frenet`
- `/planner/avoidance/otwpnts`
- `/perception/obstacles`

---

## Monitoring & Debugging

### Key Topics

```bash
# Car state
rostopic echo /car_state/odom_frenet

# Planned trajectory
rostopic echo /planner/avoidance/otwpnts

# Obstacle detections
rostopic echo /perception/obstacles

# State machine state
rostopic echo /state_machine/state
```

### Useful Commands

```bash
# Check current state
rosparam get /state_machine/current_state

# Check if race is complete
rosparam get /simulation_complete

# Force simulation complete
rosparam set /simulation_complete true
```

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Car doesn't move | Call `rosservice call /race_control/start_both` |
| Map not found | Check `stack_master/maps/` for available maps |
| Obstacle not visible | Ensure `enable_dummy_obstacle:=true` |
| Perception conflicts | Auto-disabled when using dummy obstacle |
| Wrong initial position | Check map-specific defaults or set `car_init_x/y/theta` |
