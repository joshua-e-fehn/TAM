# TAM Race Stack - Main Documentation

## Overview

The TAM Race Stack is a comprehensive autonomous racing framework for F1Tenth vehicles, supporting single-car time trials, single-car with obstacle avoidance, and multi-car racing scenarios. The stack includes multiple planning algorithms, a test automation framework, map parsing utilities, and analysis tools.

---

## Quick Reference

### Available Modes

| Mode | Description | Launch File |
|------|-------------|-------------|
| **Single Car** | Time trial / obstacle avoidance | `single_car.launch` |
| **Multi Car** | Head-to-head racing (2-4 cars) | `multi_car.launch` |
| **Test Framework** | Automated batch testing | `race_test_framework.py` |

### Available Planners

| Planner | Key | Description |
|---------|-----|-------------|
| Spliner | `spliner` | Basic spline-based overtaking planner |
| Predictive Spliner | `predictive_spliner` | GP-based opponent prediction + SQP optimization |
| TAM Sampling | `tam_sampling` | Lateral/longitudinal trajectory sampling |
| Predictive Sampler | `predictive_sampler` | Hybrid: GP prediction + TAM sampling |
| Graph-Based | `graph_based` | Discrete graph search |
| Frenet | `frenet` | Polynomial trajectory optimization |

---

## Quick Start Commands

### Single Car Mode

```bash
# Basic time trial
roslaunch stack_master single_car.launch

# With dummy obstacle for overtaking tests
roslaunch stack_master single_car.launch planner:=predictive_spliner \
    enable_dummy_obstacle:=true obstacle_speed:=0.3

# Start the race
rosservice call /race_control/start_both
```

### Multi-Car Mode

```bash
# Two cars with different planners
roslaunch stack_master multi_car.launch \
    planner_car1:=predictive_sampler \
    planner_car2:=predictive_spliner \
    global_map:=f_100%s_100%w_NUC2_mintime \
    speed_multiplier_car2:=0.6 \
    accel_multiplier_car2:=0.6

# Start the race
rosservice call /race_control/start_both
```

### Test Framework

```bash
# Run multi-car test batch
python3 src/race_stack/test_simulation/race_test_framework.py --mode multi_car --rviz true

# Run single-car without obstacle (time trial)
python3 src/race_stack/test_simulation/race_test_framework.py --mode single_car_no_obstacle --rviz false

# Run single-car with obstacle
python3 src/race_stack/test_simulation/race_test_framework.py --mode single_car_obstacle

# Run all modes sequentially
python3 src/race_stack/test_simulation/race_test_framework.py --mode all
```

### Map Parsing

```bash
# Convert TAM/Marina CSV to race stack format
python3 src/race_stack/tam_to_eth_map_parser/map_parser/basic_tam_to_eth_map_parser.py \
    src/race_stack/tam/maps/marina.csv \
    --output-name my_map \
    --scale-factor 0.12 \
    --car-name NUC2
```

### Visualization

```bash
# TAM Sampling visualization (single car)
rosrun tam_sampling_planner visualize_tam_sampling.py _single_car_mode:=true

# TAM Sampling visualization (multi-car, car1)
rosrun tam_sampling_planner visualize_tam_sampling.py _car_namespace:=car1
```

---

## Race Control Services

| Service | Description |
|---------|-------------|
| `/race_control/start_both` | Start all cars (and obstacle if enabled) |
| `/race_control/start_car1` | Start car1 only |
| `/race_control/start_car2` | Start car2 only |
| `/race_control/reset_cars` | Reset to READY state |
| `/race_control/emergency_stop` | Emergency stop all vehicles |

---

## Documentation Index

| Document | Description |
|----------|-------------|
| [README_SINGLE_CAR.md](README_SINGLE_CAR.md) | Single car mode documentation |
| [README_MULTI_CAR.md](README_MULTI_CAR.md) | Multi-car racing documentation |
| [README_TEST_FRAMEWORK.md](README_TEST_FRAMEWORK.md) | Automated test framework guide |
| [README_MAP_PARSING.md](README_MAP_PARSING.md) | Map conversion utilities |
| [README_GP_ANALYSIS.md](README_GP_ANALYSIS.md) | Gaussian Process data saving & analysis |
| [README_VISUALIZATION.md](README_VISUALIZATION.md) | Trajectory visualization tools |

---

## Directory Structure

```
race_stack/
├── stack_master/
│   ├── launch/
│   │   ├── single_car.launch      # Single-car mode
│   │   ├── multi_car.launch       # Multi-car mode
│   │   ├── base_system.launch     # Core system components
│   │   └── headtohead.launch      # Planner/controller pipeline
│   └── maps/                      # Track maps
├── planner/
│   ├── predictive-spliner/        # GP-based predictive planner
│   ├── tam_sampling_planner/      # Sampling-based planner
│   └── predictive_sampler/        # Hybrid planner
├── test_simulation/
│   ├── race_test_framework.py     # Automated testing
│   ├── configs/                   # Test configurations
│   └── logs/                      # Test results
├── tam_to_eth_map_parser/         # Map conversion tools
└── docs/                          # Documentation
```

---

## Common Parameters

### Simulation Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `sim` | `True` | Simulator (True) or physical car (False) |
| `rviz` | `True` | Enable RViz visualization |
| `map_name` / `global_map` | `f` / `M1` | Map to use |

### GP Prediction Control

| Parameter | Default | Description |
|-----------|---------|-------------|
| `use_global_prediction` | `false` | Use global waypoint prediction (testing mode) |
| `/race_test/save_gp_data` | `true` | Enable GP model data saving |

### Race Control

| Parameter | Default | Description |
|-----------|---------|-------------|
| `enable_race_start_controller` | `True` | Enable manual race start control |

---

## Troubleshooting

### Common Issues

1. **Race doesn't start**: Call `rosservice call /race_control/start_both`
2. **Missing map**: Ensure map exists in `stack_master/maps/`
3. **Perception conflicts**: When using dummy obstacle, perception is auto-disabled
4. **GP data not saving**: Set `rosparam set /race_test/save_gp_data true`

### Reset Commands

```bash
# Reset cars to starting position
rosservice call /race_control/reset_cars

# Force simulation complete (for test framework)
rosparam set /simulation_complete true

# Emergency stop
rosservice call /race_control/emergency_stop
```

---

## Author

TAM Race Stack - Autonomous Racing Research Platform
