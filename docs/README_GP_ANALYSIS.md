# GP Analysis and Data Saving - Documentation

## Overview

The Gaussian Process (GP) opponent trajectory prediction module (`gaussian_process_opp_traj.py`) can save detailed analysis data for post-race evaluation. This includes:

- **Training data**: Raw opponent detections (s, d, vs, vd)
- **Predictions**: Fitted trajectory predictions with uncertainty
- **GP models**: Kernel hyperparameters and fitted model data
- **Experiment context**: Map, obstacle parameters, prediction mode

---

## Enabling GP Data Saving

### Method 1: ROS Parameter (Recommended)

Set parameters before or during simulation:

```bash
# Enable data saving
rosparam set /race_test/save_gp_data true

# Set custom save path (optional)
rosparam set /race_test/gp_data_save_path /home/atlas/catkin_ws/gp_tests/gp_data

# Then launch simulation
roslaunch stack_master single_car.launch \
    planner:=predictive_spliner \
    enable_dummy_obstacle:=true
```

### Method 2: Launch Argument

Not directly exposed, but can be set via rosparam in launch file.

### Method 3: Private Node Parameters

Set in the predictive spliner namespace:

```bash
rosparam set /gaussian_process_opp_traj/save_gp_data true
rosparam set /gaussian_process_opp_traj/gp_data_save_path /path/to/save
```

---

## GP Data Parameters

| Parameter | Default | Namespace | Description |
|-----------|---------|-----------|-------------|
| `save_gp_data` | `True` | `~` or `/race_test/` | Enable GP data saving |
| `gp_data_save_path` | `/tmp/gp_analysis` | `~` or `/race_test/` | Directory for saved files |
| `use_gp_for_lateral` | `False` | `~` or `/race_test/` | Use GP for lateral (d) prediction |

### Prediction Mode Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `use_global_prediction` | `False` | Use global waypoint prediction instead of GP |
| `global_prediction_speed_scale` | `1.0` | Speed scaling for global prediction |
| `global_prediction_d_variance` | `0.05` | Lateral variance for global prediction |
| `global_prediction_vs_variance` | `0.05` | Speed variance for global prediction |
| `global_prediction_max_speed` | `10.0` | Max speed for global prediction |

---

## Saved Data Structure

### File Naming

Files are saved with timestamp:
```
gp_analysis_{YYYYMMDD_HHMMSS}.pkl
gp_models_{YYYYMMDD_HHMMSS}.pkl
```

### gp_analysis_{timestamp}.pkl

Main analysis data file (pickle format):

```python
{
    'timestamp': '2025-12-12T14:30:22',    # ISO format
    'track_length': 45.67,                  # meters
    
    'experiment_config': {
        'map_name': 'marina_12%s_100%w_NUC2_mintime',
        'batch_number': '20251212143022',
        'simulation_id': 'scwo_0001',
        'use_global_prediction': False,
        'use_gp_for_lateral': False,
        # Obstacle parameters...
    },
    
    'training_data': {
        's': np.array([...]),    # Arc length positions
        'd': np.array([...]),    # Lateral deviations
        'vs': np.array([...]),   # Longitudinal velocities
        'vd': np.array([...])    # Lateral velocities
    },
    
    'prediction_points': {
        's': np.array([...])     # Prediction s positions
    },
    
    'predictions': {
        'd': np.array([...]),    # Predicted lateral positions
        'vs': np.array([...]),   # Predicted velocities
        'd_ccma': np.array([...])  # CCMA-smoothed d (if available)
    },
    
    'uncertainty': {
        'sigma_d': np.array([...]),   # Lateral uncertainty
        'sigma_vs': np.array([...])   # Velocity uncertainty
    },
    
    'gp_vs_model': {
        'kernel': 'ConstantKernel * RBF + ConstantKernel * WhiteKernel',
        'kernel_params': '...',
        'log_marginal_likelihood': -123.45
    },
    
    'gp_d_model': {  # Only if use_gp_for_lateral=True
        'kernel': 'ConstantKernel * Matern + ConstantKernel * WhiteKernel',
        'kernel_params': '...',
        'log_marginal_likelihood': -98.76
    }
}
```

### gp_models_{timestamp}.pkl

Fitted model data for reconstruction:

```python
{
    'gp_vs': {
        'kernel_str': 'fitted kernel string',
        'X_train': np.array([...]),
        'y_train': np.array([...]),
        'alpha': np.array([...])  # Dual coefficients
    },
    'gp_d': {  # Only if use_gp_for_lateral=True
        'kernel_str': '...',
        'X_train': np.array([...]),
        'y_train': np.array([...]),
        'alpha': np.array([...])
    }
}
```

---

## GP Prediction Modes

### 1. Gaussian Process Mode (Default)

Uses GP regression to predict opponent trajectory:

- **Velocity prediction**: Always uses GP with RBF kernel
- **Lateral prediction**: 
  - Default: CCMA smoothing (legacy, no uncertainty)
  - Optional: GP with Matérn kernel (paper mode, provides uncertainty)

```bash
# Enable GP for lateral prediction (matches paper implementation)
rosparam set /race_test/use_gp_for_lateral true
```

### 2. Global Waypoint Prediction Mode (Testing)

Uses global waypoints for opponent prediction (deterministic, for testing):

```bash
roslaunch stack_master single_car.launch \
    use_global_prediction:=true \
    planner:=predictive_spliner
```

Or via rosparam:
```bash
rosparam set /race_test/use_global_prediction true
```

---

## Analysis Workflow

### Step 1: Run Simulation with GP Saving

```bash
# Enable GP data saving
rosparam set /race_test/save_gp_data true
rosparam set /race_test/gp_data_save_path /home/atlas/catkin_ws/gp_tests/gp_data

# Launch simulation
roslaunch stack_master single_car.launch \
    planner:=predictive_spliner \
    enable_dummy_obstacle:=true \
    obstacle_speed:=0.4

# Start race
rosservice call /race_control/start_both

# Wait for race to complete or stop manually
```

### Step 2: Load and Analyze Data

```python
import pickle
import numpy as np
import matplotlib.pyplot as plt

# Load analysis data
with open('/home/atlas/catkin_ws/gp_tests/gp_data/gp_analysis_20251212_143022.pkl', 'rb') as f:
    data = pickle.load(f)

# Extract data
train_s = data['training_data']['s']
train_d = data['training_data']['d']
pred_s = data['prediction_points']['s']
pred_d = data['predictions']['d']
sigma_d = data['uncertainty']['sigma_d']

# Plot lateral prediction with uncertainty
plt.figure(figsize=(12, 6))
plt.scatter(train_s, train_d, c='red', s=10, label='Observations')
plt.plot(pred_s, pred_d, 'b-', label='GP Prediction')
plt.fill_between(pred_s, 
                 pred_d - 2*sigma_d, 
                 pred_d + 2*sigma_d, 
                 alpha=0.3, label='95% CI')
plt.xlabel('s [m]')
plt.ylabel('d [m]')
plt.legend()
plt.title('GP Lateral Prediction')
plt.show()
```

### Step 3: Compare Multiple Runs

```python
import os
import pickle
import glob

# Find all analysis files
files = glob.glob('/home/atlas/catkin_ws/gp_tests/gp_data/gp_analysis_*.pkl')

for f in files:
    with open(f, 'rb') as file:
        data = pickle.load(file)
    
    config = data['experiment_config']
    print(f"File: {os.path.basename(f)}")
    print(f"  Map: {config.get('map_name', 'N/A')}")
    print(f"  Prediction mode: {'Global' if config.get('use_global_prediction') else 'GP'}")
    print(f"  Training points: {len(data['training_data']['s'])}")
    print()
```

---

## GP Kernel Configuration

### Velocity GP (vs)

```python
kernel_vs = ConstantKernel(0.5) * RBF(length_scale=1.0) + \
            ConstantKernel(0.2) * WhiteKernel(noise_level=1)
```

### Lateral GP (d) - When Enabled

```python
kernel_d = ConstantKernel(0.5) * Matern(length_scale=1.0, nu=3/2) + \
           ConstantKernel(0.2) * WhiteKernel(noise_level=1)
```

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Data not saved | Check `rosparam get /race_test/save_gp_data` returns True |
| Wrong save path | Check `rosparam get /race_test/gp_data_save_path` |
| No lateral uncertainty | Enable `use_gp_for_lateral` parameter |
| File not found | Check `/tmp/gp_analysis/` (default location) |
| Empty training data | Race may not have had enough opponent observations |

### Debug Commands

```bash
# Check if GP data saving is enabled
rosparam get /race_test/save_gp_data

# Check save path
rosparam get /race_test/gp_data_save_path

# Check prediction mode
rosparam get /race_test/use_global_prediction

# Check lateral GP mode
rosparam get /race_test/use_gp_for_lateral

# List saved files
ls -la /home/atlas/catkin_ws/gp_tests/gp_data/
```

---

## Integration with Test Framework

The test framework automatically sets batch number and simulation ID:

```yaml
# In test config
test_matrix:
  - simulation_id: "scwo_0001"
    planner: "predictive_spliner"
    use_global_prediction: false
    # GP data will include simulation_id and batch_number
```

Access in saved data:
```python
config = data['experiment_config']
batch = config['batch_number']  # e.g., '20251212143022'
sim_id = config['simulation_id']  # e.g., 'scwo_0001'
```

---

## Example: Complete Analysis Script

```python
#!/usr/bin/env python3
"""Analyze GP prediction data from race simulation"""

import pickle
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

def analyze_gp_data(filepath):
    """Load and visualize GP analysis data"""
    
    with open(filepath, 'rb') as f:
        data = pickle.load(f)
    
    # Extract data
    train = data['training_data']
    pred = data['predictions']
    uncertainty = data['uncertainty']
    config = data['experiment_config']
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # 1. Lateral prediction
    ax = axes[0, 0]
    ax.scatter(train['s'], train['d'], c='red', s=10, alpha=0.5, label='Observations')
    ax.plot(data['prediction_points']['s'], pred['d'], 'b-', label='GP Prediction')
    if 'sigma_d' in uncertainty and uncertainty['sigma_d'] is not None:
        ax.fill_between(data['prediction_points']['s'],
                       pred['d'] - 2*uncertainty['sigma_d'],
                       pred['d'] + 2*uncertainty['sigma_d'],
                       alpha=0.2, label='95% CI')
    ax.set_xlabel('s [m]')
    ax.set_ylabel('d [m]')
    ax.set_title('Lateral Position Prediction')
    ax.legend()
    
    # 2. Velocity prediction
    ax = axes[0, 1]
    ax.scatter(train['s'], train['vs'], c='red', s=10, alpha=0.5, label='Observations')
    ax.plot(data['prediction_points']['s'], pred['vs'], 'b-', label='GP Prediction')
    if 'sigma_vs' in uncertainty:
        ax.fill_between(data['prediction_points']['s'],
                       pred['vs'] - 2*uncertainty['sigma_vs'],
                       pred['vs'] + 2*uncertainty['sigma_vs'],
                       alpha=0.2, label='95% CI')
    ax.set_xlabel('s [m]')
    ax.set_ylabel('v [m/s]')
    ax.set_title('Velocity Prediction')
    ax.legend()
    
    # 3. Training data scatter
    ax = axes[1, 0]
    sc = ax.scatter(train['s'], train['d'], c=train['vs'], cmap='viridis', s=20)
    plt.colorbar(sc, ax=ax, label='velocity [m/s]')
    ax.set_xlabel('s [m]')
    ax.set_ylabel('d [m]')
    ax.set_title('Training Data (colored by velocity)')
    
    # 4. Info panel
    ax = axes[1, 1]
    ax.axis('off')
    info_text = f"""
Experiment Configuration
========================
Timestamp: {data['timestamp']}
Map: {config.get('map_name', 'N/A')}
Batch: {config.get('batch_number', 'N/A')}
Simulation: {config.get('simulation_id', 'N/A')}

Track length: {data['track_length']:.2f} m
Training points: {len(train['s'])}

Prediction Mode
===============
Global prediction: {config.get('use_global_prediction', False)}
GP for lateral: {config.get('use_gp_for_lateral', False)}

GP Model Info
=============
VS kernel: {data.get('gp_vs_model', {}).get('kernel', 'N/A')}
VS log-likelihood: {data.get('gp_vs_model', {}).get('log_marginal_likelihood', 'N/A'):.2f}
"""
    ax.text(0.1, 0.9, info_text, transform=ax.transAxes, 
            fontsize=10, verticalalignment='top', family='monospace')
    
    plt.tight_layout()
    plt.savefig(filepath.replace('.pkl', '_analysis.png'), dpi=150)
    plt.show()

if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        analyze_gp_data(sys.argv[1])
    else:
        print("Usage: python analyze_gp.py <path_to_pkl_file>")
```
