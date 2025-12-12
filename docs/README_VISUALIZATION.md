# Trajectory Visualization - Documentation

## Overview

The TAM Race Stack includes visualization tools for:
- **TAM Sampling Planner**: All sampled trajectories, selected trajectory, track boundaries
- **RViz Markers**: Real-time trajectory visualization in RViz
- **Matplotlib Plots**: Detailed analysis plots with `visualize_tam_sampling.py`

---

## TAM Sampling Visualization Script

### Basic Usage

```bash
# Single-car mode
rosrun tam_sampling_planner visualize_tam_sampling.py _single_car_mode:=true

# Multi-car mode (default, for car1)
rosrun tam_sampling_planner visualize_tam_sampling.py

# Multi-car mode (for car2)
rosrun tam_sampling_planner visualize_tam_sampling.py _car_namespace:=car2
```

### Script Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `~single_car_mode` | `false` | Set `true` for single-car simulation |
| `~car_namespace` | `car1` | Car namespace for multi-car mode |
| `~update_rate_hz` | `80.0` | Visualization update rate in Hz |

---

## Enabling All Sampled Trajectories Visualization

By default, the TAM sampling planner only publishes the selected trajectory. To visualize ALL sampled trajectories (valid and invalid), you need to uncomment code in the planner.

### Step 1: Edit tam_sampling_node.py

Open the file:
```bash
code src/race_stack/planner/tam_sampling_planner/src/tam_sampling_node.py
```

Find the commented section around line 1315 (in `planning_callback`):

**UNCOMMENT THIS BLOCK** (approximately lines 1315-1350):

```python
# STEP 2.5: Publish ALL sampled trajectories for visualization (before filtering)
# Access the stored raw arrays from the planner
if (hasattr(self.tam_planner, 'last_s_array') and
    self.tam_planner.last_s_array is not None and
    hasattr(self.tam_planner, 'last_n_array') and
    self.tam_planner.last_n_array is not None and
    hasattr(self.tam_planner, 'last_valid_array') and
        self.tam_planner.last_valid_array is not None):

    try:
        all_samples_markers = self.create_all_samples_markers(
            self.tam_planner.last_s_array,
            self.tam_planner.last_n_array,
            self.tam_planner.last_valid_array,
            self.track_handler
        )

        self.all_samples_pub.publish(all_samples_markers)

        # Log info about sampled trajectories
        num_total = self.tam_planner.last_s_array.shape[0]
        num_valid = np.sum(
            self.tam_planner.last_valid_array) if self.tam_planner.last_valid_array is not None else 0

    except Exception as viz_e:
        rospy.logerr_throttle(
            2, f"{self.log_name} All-samples visualization error: {viz_e}")
        import traceback
        rospy.logerr_throttle(2, traceback.format_exc())
else:
    rospy.logwarn_throttle(
        5, f"{self.log_name} Cannot publish all_samples: last_s_array={self.tam_planner.last_s_array is not None if hasattr(self.tam_planner, 'last_s_array') else 'N/A'}, last_n_array={self.tam_planner.last_n_array is not None if hasattr(self.tam_planner, 'last_n_array') else 'N/A'}")
```

### Step 2: Rebuild the Package

```bash
cd /home/atlas/catkin_ws
catkin build tam_sampling_planner
source devel/setup.bash
```

### Step 3: Launch and Visualize

```bash
# Terminal 1: Launch simulation
roslaunch stack_master single_car.launch \
    planner:=tam_sampling \
    enable_dummy_obstacle:=true

# Terminal 2: Start visualization
rosrun tam_sampling_planner visualize_tam_sampling.py _single_car_mode:=true

# Terminal 3: Start race
rosservice call /race_control/start_both
```

---

## What the Visualization Shows

### Plot 1: Track Layout with All Sampled Trajectories

- **Track boundaries**: Left (yellow) and right (orange) boundaries
- **Centerline**: Green dashed line
- **Racing line**: Blue dashed line
- **All sampled trajectories**: 
  - Blue = valid trajectories
  - Red = invalid trajectories (collision/boundary violation)
- **Selected trajectory**: Thick green line
- **Current vehicle position**: Red circle with heading indicator
- **Start/finish line**: Checkered pattern at s=0

### Plot 2: Frenet Trajectories (s-n space)

- Shows trajectories in Frenet coordinates
- Track boundaries as functions of s
- Current position indicator
- Start/finish line

### Plot 3: Sampling Statistics

- Number of global waypoints
- Current position (s, n, velocity)
- Number of sampled trajectories
- Number of valid trajectories
- ROS parameter status

---

## Visualization Topics

### Published by TAM Sampling Planner

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `planner/avoidance/markers` | `MarkerArray` | Selected trajectory markers |
| `planner/avoidance/all_samples` | `MarkerArray` | All sampled trajectories (if enabled) |
| `planner/avoidance/otwpnts` | `OTWpntArray` | Trajectory waypoints for controller |

### Subscribed by Visualization Script

| Topic | Single-Car | Multi-Car | Description |
|-------|------------|-----------|-------------|
| Global waypoints | `/global_waypoints` | `/{car}/global_waypoints` | Track reference |
| Frenet odometry | `/car_state/odom_frenet` | `/{car}/car_state/odom_frenet` | Vehicle state |
| Markers | `/planner/avoidance/markers` | `/{car}/planner/avoidance/markers` | Selected trajectory |
| All samples | `/planner/avoidance/all_samples` | `/{car}/planner/avoidance/all_samples` | All trajectories |

---

## RViz Visualization

The standard RViz display also shows trajectory markers:

### Adding Marker Displays

1. Open RViz (launched with simulation)
2. Click "Add" → "By topic"
3. Add these marker topics:
   - `/planner/avoidance/markers` or `/{car}/planner/avoidance/markers`
   - `/planner/avoidance/all_samples` (if enabled)
   - `/perception/obstacles/markers`

### Color Coding in RViz

**Planned Trajectory Colors** (per car):
- car1: Bright Red (1.0, 0.0, 0.0)
- car2: Bright Blue (0.0, 0.5, 1.0)
- car3: Bright Green (0.0, 1.0, 0.0)
- car4: Bright Yellow (1.0, 1.0, 0.0)

**All Samples Colors**:
- Valid trajectories: Blue (0.0, 0.5, 1.0)
- Invalid trajectories: Red (1.0, 0.0, 0.0)
- Transparency: 0.5 alpha for readability

---

## Example Workflows

### Single-Car Visualization

```bash
# Terminal 1: Launch simulation with TAM sampling
roslaunch stack_master single_car.launch \
    planner:=tam_sampling \
    enable_dummy_obstacle:=true \
    obstacle_speed:=0.3

# Terminal 2: Start matplotlib visualization
rosrun tam_sampling_planner visualize_tam_sampling.py _single_car_mode:=true

# Terminal 3: Start the race
rosservice call /race_control/start_both
```

### Multi-Car Visualization

```bash
# Terminal 1: Launch multi-car simulation
roslaunch stack_master multi_car.launch \
    planner_car1:=tam_sampling \
    planner_car2:=spliner

# Terminal 2: Visualize car1
rosrun tam_sampling_planner visualize_tam_sampling.py _car_namespace:=car1

# Terminal 3: (Optional) Visualize car2
rosrun tam_sampling_planner visualize_tam_sampling.py _car_namespace:=car2

# Terminal 4: Start race
rosservice call /race_control/start_both
```

### Predictive Sampler Visualization

The predictive sampler (hybrid) uses TAM sampling for trajectory generation, so the same visualization works:

```bash
# Launch with predictive sampler
roslaunch stack_master single_car.launch \
    planner:=predictive_sampler \
    enable_dummy_obstacle:=true

# Visualize
rosrun tam_sampling_planner visualize_tam_sampling.py _single_car_mode:=true
```

---

## Performance Considerations

### Update Rate

The visualization script runs at 80 Hz by default. For slower machines:

```bash
# Reduce update rate
rosrun tam_sampling_planner visualize_tam_sampling.py \
    _single_car_mode:=true \
    _update_rate_hz:=30.0
```

### All Samples Publishing

Publishing all sampled trajectories adds overhead. For performance-critical runs:
- Keep the code commented out in `tam_sampling_node.py`
- Use RViz with only selected trajectory markers

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| No plot appears | Check matplotlib backend: `export MPLBACKEND=TkAgg` |
| Empty track | Wait for global waypoints to be published |
| No trajectories | Ensure planner is running and publishing |
| All samples empty | Uncomment code block in `tam_sampling_node.py` |
| Wrong namespace | Check `_car_namespace` or `_single_car_mode` |
| Slow visualization | Reduce `_update_rate_hz` |

### Debug Commands

```bash
# Check if all_samples is being published
rostopic hz /planner/avoidance/all_samples

# Check message content
rostopic echo /planner/avoidance/all_samples -n 1

# Check global waypoints
rostopic echo /global_waypoints -n 1

# List all planner topics
rostopic list | grep planner
```

---

## Code Reference

### Visualization Script Location

```
src/race_stack/planner/tam_sampling_planner/scripts/visualize_tam_sampling.py
```

### TAM Sampling Node Location

```
src/race_stack/planner/tam_sampling_planner/src/tam_sampling_node.py
```

### Key Methods in tam_sampling_node.py

| Method | Description |
|--------|-------------|
| `create_all_samples_markers()` | Creates MarkerArray for all sampled trajectories |
| `create_f1tenth_visualization_markers()` | Creates markers for selected trajectory |
| `planning_callback()` | Main planning loop (contains visualization code) |

---

## Summary: Quick Enable Steps

1. **Edit**: `src/race_stack/planner/tam_sampling_planner/src/tam_sampling_node.py`
2. **Uncomment**: Lines ~1315-1350 (STEP 2.5 block)
3. **Rebuild**: `catkin build tam_sampling_planner && source devel/setup.bash`
4. **Launch**: `roslaunch stack_master single_car.launch planner:=tam_sampling`
5. **Visualize**: `rosrun tam_sampling_planner visualize_tam_sampling.py _single_car_mode:=true`
6. **Start**: `rosservice call /race_control/start_both`
