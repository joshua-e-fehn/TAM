# Test Framework - Documentation

## Overview

The Race Test Framework (`race_test_framework.py`) is an automated testing system that runs multiple race simulations sequentially with different configurations. It supports:

- **Batch testing** across multiple planner configurations
- **Three test modes**: single car (no obstacle), single car (with obstacle), multi-car
- **Automated result logging** with structured output directories
- **Early stopping** based on completion patterns

---

## Basic Usage

```bash
# Navigate to workspace
cd /home/atlas/catkin_ws

# Run multi-car tests (default)
python3 src/race_stack/test_simulation/race_test_framework.py --mode multi_car

# Run single-car time trial tests
python3 src/race_stack/test_simulation/race_test_framework.py --mode single_car_no_obstacle

# Run single-car with obstacle tests
python3 src/race_stack/test_simulation/race_test_framework.py --mode single_car_obstacle

# Run all test modes sequentially
python3 src/race_stack/test_simulation/race_test_framework.py --mode all
```

---

## Command Line Arguments

| Argument | Values | Default | Description |
|----------|--------|---------|-------------|
| `--mode` | `single_car_no_obstacle`, `single_car_obstacle`, `multi_car`, `all` | `multi_car` | Test mode to run |
| `--config` | Path to YAML | Mode-specific | Custom config file (overrides mode-based selection) |
| `--rviz` | `true`, `false` | `true` | Enable RViz visualization |

### Examples

```bash
# Run without visualization (faster, for automated runs)
python3 src/race_stack/test_simulation/race_test_framework.py --mode multi_car --rviz false

# Use custom config file
python3 src/race_stack/test_simulation/race_test_framework.py --mode single_car_obstacle --config my_custom_config.yaml

# Run all modes with visualization
python3 src/race_stack/test_simulation/race_test_framework.py --mode all --rviz true
```

---

## Test Modes

### 1. Single Car No Obstacle (`single_car_no_obstacle`)

Time trial mode - tests pure lap performance.

**Config file**: `single_car_no_obstacle_config.yaml`

**Use cases**:
- Lap time optimization
- Planner speed comparison
- Baseline performance measurement

### 2. Single Car With Obstacle (`single_car_obstacle`)

Overtaking mode - tests avoidance and overtaking with dummy obstacle.

**Config file**: `single_car_with_obstacle_config.yaml`

**Use cases**:
- Overtaking algorithm testing
- Collision avoidance validation
- Obstacle speed sensitivity analysis

### 3. Multi-Car (`multi_car`)

Head-to-head racing - tests real racing scenarios.

**Config file**: `multi_car_config.yaml`

**Use cases**:
- Algorithm comparison (different planners per car)
- Overtaking strategy evaluation
- Multi-agent coordination testing

### 4. All (`all`)

Runs all three modes sequentially:
1. Single car no obstacle
2. Single car with obstacle
3. Multi-car

---

## Configuration Files

### Config File Location

```
src/race_stack/test_simulation/
├── single_car_no_obstacle_config.yaml
├── single_car_with_obstacle_config.yaml
├── multi_car_config.yaml
└── race_test_default_config.yaml
```

### Test Matrix Format

Each config file contains a `test_matrix` list of test configurations:

```yaml
test_matrix:
  - simulation_id: "mult_0001"
    name: "spliner_vs_predictive_spliner"
    mode: "multi_car"
    planner_car1: "predictive_spliner"
    planner_car2: "predictive_spliner"
    speed_multiplier_car1: 1.0
    speed_multiplier_car2: 0.5
    global_map: "my_map_20%s_100%w_NUC2_mintime"
    use_global_prediction: false
    show_simulation_output: false
    
    # Planner-specific parameters
    car1_predictive_spliner_lookahead_dist: 20.0
    car1_predictive_spliner_max_v: 10.0
    car1_predictive_spliner_evasion_dist: 0.6
    # ... more parameters
```

### Default Configuration

`race_test_default_config.yaml` defines race completion criteria:

```yaml
# Collision detection parameters
warning_distance: 1.5      # meters
critical_distance: 0.8     # meters
collision_distance: 0.4    # meters

# Monitoring parameters
check_rate: 144.0          # Hz

# Race completion parameters
target_laps: 3
overtake_lead_distance: 5.0  # meters

# Event count thresholds
max_overtakes: 1           # Single car obstacle mode
max_boundary_collisions: 1
max_car_collisions: 1      # Multi-car mode

# Boundary collision behavior
end_race_on_boundary_collision: true
```

---

## Test Configuration Parameters

### Common Parameters

| Parameter | Description |
|-----------|-------------|
| `simulation_id` | Unique ID for the test |
| `name` | Descriptive name |
| `mode` | `single_car_obstacle`, `single_car_no_obstacle`, or `multi_car` |
| `global_map` | Map name |
| `show_simulation_output` | Show launch output |
| `use_global_prediction` | Use global waypoint prediction |

### Single Car With Obstacle Parameters

| Parameter | Description |
|-----------|-------------|
| `planner` | Planner for ego car |
| `obstacle_speed` | Obstacle speed scaler (0.0-1.0) |
| `obstacle_start_s` | Obstacle starting position (meters) |

### Multi-Car Parameters

| Parameter | Description |
|-----------|-------------|
| `planner_car1` | Planner for car 1 |
| `planner_car2` | Planner for car 2 |
| `speed_multiplier_car1` | Speed multiplier for car 1 |
| `speed_multiplier_car2` | Speed multiplier for car 2 |
| `accel_multiplier_car1` | Accel multiplier for car 1 |
| `accel_multiplier_car2` | Accel multiplier for car 2 |

### Planner-Specific Parameters

Parameters are prefixed with car name (multi-car) or plain (single-car):

```yaml
# Multi-car: car1 predictive spliner
car1_predictive_spliner_lookahead_dist: 20.0
car1_predictive_spliner_max_v: 10.0
car1_predictive_spliner_evasion_dist: 0.6

# Single-car: predictive spliner
predictive_spliner_lookahead_dist: 20.0
predictive_spliner_max_v: 10.0
predictive_spliner_evasion_dist: 0.6
```

---

## Output and Logging

### Log Directory Structure

```
src/race_stack/test_simulation/logs/
├── multi_car/
│   └── {map}/
│       └── {planner1}/{planner2}/
│           └── batch_{timestamp}/
│               ├── race_test_default_config.yaml
│               ├── multi_car_config.yaml
│               └── {simulation_id}/
│                   ├── race_log.yaml
│                   └── events.log
├── single_car_obstacle/
│   └── {map}/
│       └── {planner}/
│           └── batch_{timestamp}/
│               └── {simulation_id}/
│                   └── ...
└── single_car_no_obstacle/
    └── ...
```

### Batch Number

Each test run generates a unique batch number based on timestamp:
- Format: `YYYYMMDDHHMMSS`
- Example: `20251212143022`

Set as ROS parameter: `/race_test/batch_number`

---

## Race Completion Criteria

A simulation ends when any of these conditions are met:

1. **Target laps completed** (`target_laps`)
2. **Maximum overtakes reached** (`max_overtakes`) - single car obstacle mode
3. **Boundary collision** (`max_boundary_collisions`)
4. **Car collision** (`max_car_collisions`) - multi-car mode
5. **Timeout** (default: 1200 seconds)
6. **Manual completion**: `rosparam set /simulation_complete true`

---

## Early Stopping

The framework includes automatic early stopping:
- If last 3 consecutive tests end with same completion reason (e.g., all lap completions)
- Helps avoid running redundant tests

---

## Runtime Control

### Manual Simulation Control

```bash
# Force current simulation to complete
rosparam set /simulation_complete true

# Check simulation status
rosparam get /race_test/simulation_complete

# Check batch number
rosparam get /race_test/batch_number
```

### Interrupting Tests

Press `Ctrl+C` to interrupt the test framework:
- Current simulation will be terminated
- Results up to that point are saved
- Summary is printed

---

## Creating Custom Configurations

### Example: Single Car Obstacle Config

```yaml
# my_custom_obstacle_config.yaml
test_matrix:
  - simulation_id: "custom_001"
    name: "predictive_spliner_slow_obstacle"
    mode: "single_car_obstacle"
    planner: "predictive_spliner"
    global_map: "f_100%s_100%w_NUC2_mintime"
    obstacle_speed: 0.3
    obstacle_start_s: 0
    use_global_prediction: false
    show_simulation_output: true
    
    predictive_spliner_lookahead_dist: 25.0
    predictive_spliner_max_v: 8.0
    predictive_spliner_evasion_dist: 0.5

  - simulation_id: "custom_002"
    name: "predictive_spliner_fast_obstacle"
    mode: "single_car_obstacle"
    planner: "predictive_spliner"
    global_map: "f_100%s_100%w_NUC2_mintime"
    obstacle_speed: 0.6
    obstacle_start_s: 0
    use_global_prediction: false
    
    predictive_spliner_lookahead_dist: 25.0
    predictive_spliner_max_v: 8.0
```

### Running Custom Config

```bash
python3 src/race_stack/test_simulation/race_test_framework.py \
    --mode single_car_obstacle \
    --config my_custom_obstacle_config.yaml
```

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Config file not found | Ensure file is in `test_simulation/` directory |
| Tests hang | Check for ROS node crashes, try `--rviz false` |
| No logs generated | Check write permissions in logs directory |
| Wrong mode parameters | Ensure `mode` field matches command line mode |
| Tests fail immediately | Check map exists in `stack_master/maps/` |

### Debug Tips

```bash
# Run with simulation output visible
# Set in config: show_simulation_output: true

# Check ROS master
rostopic list

# Check for running nodes
rosnode list

# Kill stuck processes
pkill -9 roslaunch
pkill -9 gzserver
```

---

## Analysis Tools

After running tests, analyze results with:

```bash
# Analyze race logs
python3 src/race_stack/test_simulation/analyze_race_logs.py

# Plot failure positions
python3 src/race_stack/test_simulation/plot_failure_positions.py

# Plot success positions
python3 src/race_stack/test_simulation/plot_success_positions.py
```
