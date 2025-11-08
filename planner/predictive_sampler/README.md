# Predictive Sampler Planner

## Overview

The **Predictive Sampler** is a hybrid overtaking planner that combines:
- **State Transitions**: From Predictive Spliner (PSSAMP functions)
- **Trajectory Generation**: From TAM Sampling (sampling-based optimization)
- **Conditional Planning**: Performance optimization that only runs expensive computations when needed

This configuration provides the strategic intelligence of Predictive Spliner with the robust trajectory generation capabilities of TAM Sampling.

---

## Architecture

### Components

1. **State Machine** (`state_machine_node`)
   - Uses Predictive Spliner state transition logic (PSSAMP functions)
   - Manages states: READY → GB_TRACK → TRAILING → OVERTAKE
   - Publishes appropriate waypoints based on current state

2. **TAM Sampling Planner** (`tam_sampling_node`)
   - **Mode**: `predictive_sampler` (conditional planning)
   - Generates overtaking trajectories using sampling-based optimization
   - Only plans when conditions are met (see Conditional Planning below)

3. **Collision Prediction** (`collision_prediction_node`)
   - Forward simulates 4-second collision horizon
   - Publishes Regions of Collision (ROCs) to `collision_prediction/obstacles`
   - Essential for TAM's obstacle avoidance

4. **Gaussian Process Opponent Trajectory** (`gp_opponent_trajectory_node`)
   - ML-based prediction of opponent's future raceline
   - Uses sklearn GaussianProcessRegressor
   - Provides opponent trajectory to collision prediction

### What's NOT Needed

- **SQP Avoidance Node**: Predictive Sampler uses TAM's trajectory generation instead of spline-based SQP optimization
- **Frenet Conversion**: TAM operates in global coordinates, doesn't need Frenet conversions

---

## Conditional Planning Mode

### Concept

In `predictive_sampler` mode, TAM Sampling only executes the full planning cycle when specific conditions are met. This mirrors the Predictive Spliner's SQP node behavior:

**Planning is ACTIVE when:**
1. ✅ Vehicle is in overtaking sector (`ot_section_check == True`)
2. ✅ At least one obstacle detected within 15m lookahead distance
3. ✅ At least one obstacle within 2m lateral distance from ego trajectory

**Planning is INACTIVE when:**
- ❌ Not in overtaking sector
- ❌ No obstacles nearby
- ❌ All obstacles too far laterally

When inactive, the node publishes an **empty trajectory** (`OTWpntArray` with no waypoints), signaling the state machine to fall back to default behavior.

### Performance Benefits

- **Reduced CPU Usage**: Avoids expensive sampling optimization when not needed
- **Improved Responsiveness**: More resources available for critical perception/control tasks
- **Battery Efficiency**: Important for physical F1Tenth cars with limited onboard compute

### Implementation Details

See `tam_sampling_node.py`:
- **Parameter**: `planning_mode` (set to `"predictive_sampler"`)
- **Method**: `_check_should_plan()` (lines ~637-691)
- **Integration**: `run_planning_cycle()` early returns if check fails (lines ~1258-1285)

---

## Launch File

### Usage

```bash
# Launch Predictive Sampler for single vehicle
roslaunch predictive_sampler predictive_sampler.launch veh_name:=sim1 opponent_name:=sim2

# With custom TAM config
roslaunch predictive_sampler predictive_sampler.launch \
    veh_name:=sim1 \
    tam_config:=$(rospack find tam_sampling_planner)/config/custom_tam.yaml
```

### Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `veh_name` | `sim1` | Namespace for ego vehicle |
| `opponent_name` | `sim2` | Namespace for opponent vehicle |
| `tam_config` | `tam_config.yaml` | TAM Sampling configuration file |
| `tam_mode` | `predictive_sampler` | Planning mode (`predictive_sampler` or `tam_sampling`) |
| `collision_pred_config` | `collision_prediction_config.yaml` | Collision prediction settings |
| `gp_config` | `gp_opponent_trajectory_config.yaml` | Gaussian Process settings |
| `state_machine_config` | `state_machine.yaml` | State machine configuration |

### Key Parameters

**State Machine:**
```yaml
ot_planner: "predictive_sampler"  # CRITICAL: Enables PSSAMP transitions
```

**TAM Sampling:**
```yaml
planning_mode: "predictive_sampler"  # Enables conditional planning
lookahead: 15.0                      # Obstacle detection range (m)
obs_traj_thresh: 2.0                 # Lateral obstacle threshold (m)
```

---

## ROS Topics

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/<veh_name>/planner/global_wpts` | `WpntArray` | Global racing line |
| `/<veh_name>/collision_prediction/obstacles` | `ObstacleArray` | ROCs from collision prediction |
| `/<opponent_name>/particle_filter/pose` | `PoseStamped` | Opponent pose |
| `/<veh_name>/ot_section_check` | `Bool` | Whether in overtaking sector |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/<veh_name>/planner/avoidance/otwpnts` | `OTWpntArray` | Overtaking trajectories (or empty) |
| `/<veh_name>/controller/wpnts` | `WpntArray` | Final waypoints to controller (from state machine) |
| `/<veh_name>/collision_prediction/opponent_trajectory` | `OpponentTrajectory` | Predicted opponent path |

---

## State Transitions

### State Flow

```
READY
  ↓
GB_TRACK (Global Tracking)
  ↓ (opponent detected + close + in_ot_sector)
TRAILING (Follow opponent, plan conditionally)
  ↓ (good overtaking opportunity)
OVERTAKE (Execute TAM trajectory)
  ↓ (overtake complete)
GB_TRACK
```

### Transition Functions (from `state_transitions.py`)

- **PSAMPReadyTransition**: READY → GB_TRACK when vehicle initialized
- **PSSAMPGlobalTrackingTransition**: GB_TRACK → TRAILING when opponent close
- **PSSAMPTrailingTransition**: TRAILING ↔ OVERTAKE based on trajectory quality
- **PSSAMPSamplingTransition**: OVERTAKE → GB_TRACK when overtake complete

These are the **same transitions** used by Predictive Spliner, ensuring consistent strategic behavior.

---

## Comparison with Other Planners

| Feature | Predictive Spliner | TAM Sampling | **Predictive Sampler** |
|---------|-------------------|--------------|------------------------|
| State Transitions | PSSAMP functions | TSAMP functions | **PSSAMP functions** |
| Trajectory Generation | SQP (spline-based) | Sampling optimization | **Sampling optimization** |
| Conditional Planning | ✅ (SQP only plans when needed) | ❌ (always plans) | **✅ (TAM plans when needed)** |
| Gaussian Process | ✅ | ❌ | **✅** |
| Collision Prediction | ✅ | ✅ | **✅** |
| Computational Cost | Medium | High | **Medium (conditional)** |

**Key Insight**: Predictive Sampler achieves the computational efficiency of Predictive Spliner while using TAM's more robust trajectory generation algorithm.

---

## Testing

### Verify Conditional Planning

```bash
# Terminal 1: Launch Predictive Sampler
roslaunch predictive_sampler predictive_sampler.launch

# Terminal 2: Monitor TAM planning activity
rostopic echo /sim1/planner/avoidance/otwpnts

# Terminal 3: Check ot_section_check status
rostopic echo /sim1/ot_section_check

# Expected Behavior:
# - When ot_section_check=False: Empty trajectories published
# - When ot_section_check=True + obstacles nearby: Full trajectories published
```

### Verify State Transitions

```bash
# Monitor state machine
rostopic echo /sim1/state_machine/current_state

# Expected: READY → GB_TRACK → TRAILING → OVERTAKE → GB_TRACK
```

### Performance Monitoring

```bash
# CPU usage (should be lower than continuous TAM sampling)
top -p $(pgrep -f tam_sampling_node)

# Planning rate
rostopic hz /sim1/planner/avoidance/otwpnts
```

---

## Troubleshooting

### No Trajectories Published

**Symptoms**: `/planner/avoidance/otwpnts` shows empty arrays even with obstacles nearby

**Causes**:
1. `ot_section_check` topic not publishing → Check state machine launch
2. Obstacles outside threshold → Check `lookahead` and `obs_traj_thresh` parameters
3. TAM node crashed → Check `rosnode list` and logs

**Fix**:
```bash
# Verify topics exist
rostopic list | grep -E "(ot_section_check|collision_prediction|avoidance)"

# Check TAM node logs
rosnode info /sim1/tam_sampling_planner
```

### Wrong State Transitions

**Symptoms**: State machine not entering OVERTAKE state

**Causes**:
1. `ot_planner` parameter not set to `predictive_sampler`
2. Trajectory quality too low for overtake threshold

**Fix**:
```bash
# Verify planner parameter
rosparam get /sim1/state_machine/ot_planner

# Should output: "predictive_sampler"
```

### Continuous Planning (Not Conditional)

**Symptoms**: TAM always plans, even when no obstacles

**Causes**:
1. `planning_mode` parameter not set to `predictive_sampler`
2. `ot_section_check` subscriber not receiving messages

**Fix**:
```bash
# Verify planning mode
rosparam get /sim1/tam_sampling_planner/planning_mode

# Check subscriber connection
rostopic info /sim1/ot_section_check
```

---

## Development Notes

### Files Modified

1. **State Machine Integration**:
   - `state_machine/src/states.py`: Added `predictive_sampler` to Trailing/Overtaking states
   - `state_machine/src/state_machine_node.py`: Added PSSAMP mapping for predictive_sampler

2. **TAM Sampling Conditional Mode**:
   - `tam_sampling_planner/src/tam_sampling_node.py`:
     * Added `planning_mode` parameter detection
     * Added `ot_section_check` subscriber
     * Implemented `_check_should_plan()` method
     * Modified `run_planning_cycle()` to use conditional check

3. **Package Structure**:
   - `predictive_sampler/launch/predictive_sampler.launch`: Main launch file
   - `predictive_sampler/README.md`: This documentation

### Future Enhancements

- [ ] Add RViz visualization config for trajectory debugging
- [ ] Implement adaptive thresholds based on track curvature
- [ ] Add performance metrics logging (planning time, CPU usage)
- [ ] Create multi-opponent support (currently assumes single opponent)
- [ ] Add recovery behavior when all trajectories infeasible

---

## References

- **TAM Sampling Planner**: `/src/race_stack/planner/tam_sampling_planner/`
- **Predictive Spliner**: `/src/race_stack/planner/predictive_spliner/`
- **State Machine**: `/src/race_stack/state_machine/`
- **Collision Prediction**: `/src/race_stack/planner/predictive_spliner/src/collision_prediction_node.py`
- **Gaussian Process**: `/src/race_stack/planner/predictive_spliner/src/gp_opponent_trajectory_node.py`

---

## License

Same license as the parent F1Tenth racing stack.

---

**Author**: Implementation based on user requirements for hybrid Predictive Spliner + TAM Sampling approach  
**Last Updated**: 2025

- **Integration**: `run_planning_cycle()` early returns if check fails (lines ~1258-1285)
- TAM Sampling parameters (trajectory generation, safety margins, etc.)

## Integration

The planner is integrated into the state machine via:

1. **State Transitions**: Uses `PSSAMP*Transition` functions in `state_transitions.py`
2. **Waypoint Generation**: 
   - GB_TRACK/TRAILING: Uses `get_splini_wpts()` 
   - OVERTAKE: Uses TAM trajectory via `get_splini_wpts()` (TAM fusion)
3. **Subscribers**: Same as Predictive Spliner (merger, force_trailing, avoidance_wpnts)

## Usage

This planner is ideal for scenarios where:
- Predictive state transitions are desired (predictive collision avoidance)
- TAM's sampling-based overtaking is preferred over spline-based overtaking
- You want the safety of predictive planning with the flexibility of sampling

## Development Status

Initial implementation complete. The planner reuses existing infrastructure and requires no custom nodes - it's purely a configuration option in the state machine.
