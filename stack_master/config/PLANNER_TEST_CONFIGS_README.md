# Planner Test Configuration Documentation

This directory contains configuration files for testing the spliner and predictive_spliner planners with obstacles.

## Files

### 1. planner_params_variable.yaml

This file defines all tunable parameters for both planners with their metadata:
- **default**: The default value used in standard operation
- **min**: The minimum allowed value
- **max**: The maximum allowed value
- **description**: What the parameter does
- **unit**: The unit of measurement

This file serves as the reference for understanding parameter ranges and is used to generate test configurations.

### 2. single_car_with_obstacles_config.yaml

This file contains actual test run configurations. Each configuration is a complete parameter set that can be used to run a test.

**Naming Convention**: `{planner_name}_{param_name}` or `{planner_name}_{description}`

## Test Configuration Strategy

The test configurations use a **broad sampling approach** with varied sampling rates based on parameter importance:

### Parameter Importance Categories

1. **HIGH Importance** (Safety-critical or major performance impact)
   - Sampled at: min, low (25%), high (75%), max
   - Examples: evasion_dist, obs_traj_tresh, spline_bound_mindist, safety distances, lookahead_dist
   
2. **MEDIUM Importance** (Significant tuning parameters)
   - Sampled at: min, default, max
   - Examples: pre/post apex distances, prediction parameters, velocity limits, avoidance resolution
   
3. **LOW Importance** (Fine-tuning parameters)
   - Sampled at: min, default, max
   - Examples: fixed_pred_time, max_expire_counter

### Test Configuration Types

Each planner has three types of test configurations:

1. **Baseline**: All parameters set to defaults
2. **Single-parameter variations**: One parameter varied while others stay at default
3. **Combined configurations**: Multiple parameters adjusted together
   - **Aggressive**: Tighter margins, faster reactions, higher performance
   - **Conservative**: Larger margins, safer operation, lower performance
   - **Balanced**: Middle ground between aggressive and conservative

## Spliner Test Configurations

Total configurations: 28

- 1 baseline
- 24 single-parameter variations covering:
  - evasion_dist (4 samples)
  - obs_traj_tresh (4 samples)
  - spline_bound_mindist (4 samples)
  - pre_apex distances (3 samples)
  - post_apex distances (3 samples)
  - kd_obs_pred (3 samples)
  - fixed_pred_time (3 samples)
- 3 combined configurations (aggressive/conservative/balanced)

## Predictive Spliner Test Configurations

Total configurations: 39

- 1 baseline
- 35 single-parameter variations covering:
  - n_time_steps (3 samples)
  - dt (3 samples)
  - save_distance (4 samples)
  - max_v (3 samples)
  - max_a (3 samples)
  - evasion_dist_sqp (4 samples)
  - lookahead_dist (4 samples)
  - avoidance_resolution (3 samples)
  - back_to_raceline timing (3 samples)
- 3 combined configurations (aggressive/conservative/balanced)

## Usage

### Running a Test Configuration

To run a specific test configuration with the single_car.launch:

```bash
# Example: Run spliner with aggressive configuration
roslaunch stack_master single_car.launch \
  planner:=spliner \
  enable_dummy_obstacle:=true \
  obstacle_speed:=0.5

# Then apply the parameters via dynamic reconfigure or ROS parameters
```

### Loading Configuration Programmatically

```python
import yaml

# Load the configurations
with open('single_car_with_obstacles_config.yaml', 'r') as f:
    configs = yaml.safe_load(f)

# Access a specific configuration
spliner_aggressive = configs['spliner_aggressive']
print(f"Evasion distance: {spliner_aggressive['evasion_dist']}")

# Iterate through all spliner configurations
for config_name, params in configs.items():
    if config_name.startswith('spliner_'):
        print(f"Running test: {config_name}")
        # Apply parameters and run test
```

### Integration with BayesOpt

These configurations can serve as:
1. **Initial sampling points** for Bayesian optimization
2. **Validation set** to test optimized parameters
3. **Reference configurations** to understand parameter ranges

## Next Steps

After running these broad test configurations:

1. Analyze the results to identify promising parameter regions
2. Create refined configurations sampling more densely near the best-performing regions
3. Use Bayesian optimization for final fine-tuning within the best regions
4. Document the final optimized parameters

## Notes

- Boolean parameters (update_waypoints, avoid_static_obs) are not included in variation tests as they are typically binary choices
- Some parameter combinations may be incompatible (e.g., very low n_time_steps with very high dt) - filter these during execution
- All configurations maintain physical constraints (e.g., min_a <= max_a, min_v <= max_v)
