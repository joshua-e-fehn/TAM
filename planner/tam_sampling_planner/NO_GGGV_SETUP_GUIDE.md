# No GGGV Diagrams? No Problem! - Setup Guide

When you don't have GGGV (G-G-G-V) diagrams for your vehicle, you can still use the TAM sampling planner with conservative estimates. This guide shows you how to set up and tune the system.

## What are GGGV Diagrams?

GGGV diagrams define your vehicle's acceleration envelope - the maximum longitudinal and lateral accelerations your car can achieve at different speeds. They're typically generated through:
- Vehicle testing on a skid pad
- Professional vehicle dynamics analysis
- Tire testing data
- Simulation with detailed vehicle models

## Option 1: Use the Simple GGGV Manager (Recommended)

The TAM sampling planner now includes a fallback `SimpleGGGVManager` that provides conservative estimates based on typical racing car parameters.

### Step 1: Configure Parameters

Edit the configuration file: `config/simple_gggv_config.yaml`

```yaml
# Start with these conservative values for a typical racing car
max_longitudinal_accel: 8.0   # Forward acceleration [m/s²]
max_longitudinal_decel: -12.0 # Braking [m/s²] 
max_lateral_accel: 15.0       # Cornering [m/s²]
```

### Step 2: Vehicle-Specific Tuning

Choose parameters based on your vehicle type:

#### F1/10 Scale Car (1:10 scale):
```yaml
max_longitudinal_accel: 4.0
max_longitudinal_decel: -6.0  
max_lateral_accel: 8.0
```

#### Go-Kart:
```yaml
max_longitudinal_accel: 5.0
max_longitudinal_decel: -8.0
max_lateral_accel: 10.0
```

#### Road Car:
```yaml
max_longitudinal_accel: 4.0
max_longitudinal_decel: -7.0
max_lateral_accel: 7.0
```

#### High-Performance Sports Car:
```yaml
max_longitudinal_accel: 6.0
max_longitudinal_decel: -10.0
max_lateral_accel: 12.0
```

### Step 3: Test and Iterate

1. **Start Conservative**: Begin with lower values than you think your car can achieve
2. **Observe Behavior**: 
   - Too slow/cautious? → Increase limits gradually
   - Vehicle unstable/sliding? → Decrease limits
3. **Test Incrementally**: Change one parameter at a time
4. **Validate**: Record actual accelerations during driving and compare

## Option 2: Estimate from Basic Testing

If you can do some basic testing, you can get better estimates:

### Simple Acceleration Test:
1. **Straight Line Acceleration**: 
   - Accelerate from 0-30 mph, measure time
   - Calculate: `max_accel = velocity_change / time`
   
2. **Braking Test**:
   - Brake from 30 mph to 0, measure distance
   - Calculate: `max_decel = velocity²/(2×distance)`

3. **Cornering Test** (SAFE LOCATION ONLY):  
   - Drive constant speed circle, gradually increase speed until limit
   - Calculate: `lateral_accel = velocity²/radius`

### Example Calculation:
```python
# If your car accelerates 0-30 mph (13.4 m/s) in 3 seconds:
max_accel = 13.4 / 3.0 = 4.47 m/s²

# If it stops from 30 mph in 20 meters:
max_decel = (13.4²) / (2 × 20) = 4.49 m/s²

# If it can corner at 20 mph (8.94 m/s) in 15m radius:
lateral_accel = (8.94²) / 15 = 5.33 m/s²
```

## Option 3: Use Literature Values

Find published specifications for similar vehicles:

### Typical Acceleration Values by Vehicle Type:

| Vehicle Type | Long. Accel | Long. Decel | Lateral Accel |
|--------------|-------------|-------------|---------------|
| Road Car     | 3-5 m/s²    | -5 to -8 m/s² | 5-8 m/s²    |
| Sports Car   | 5-8 m/s²    | -8 to -12 m/s² | 8-12 m/s²   |
| Formula Car  | 8-12 m/s²   | -12 to -18 m/s² | 12-18 m/s² |
| F1 Car       | 10-15 m/s²  | -15 to -20 m/s² | 15-25 m/s² |
| Go-Kart      | 4-7 m/s²    | -6 to -10 m/s² | 8-15 m/s²   |

## Implementation Steps

### 1. Update Your Code:

The longitudinal sampling module will automatically detect if GGGV diagrams are available and fall back to the simple manager:

```python
# This happens automatically - no code changes needed!
try:
    from planning_common.track.gggvManager import GGGVManager
except ImportError:
    from simple_gggv_manager import SimpleGGGVManager as GGGVManager
    # Uses conservative estimates
```

### 2. Parameter File Setup:

Create or modify your parameter file:

```yaml
# In your ROS parameter file
tam_sampling_planner:
  gggv:
    max_longitudinal_accel: 6.0    # Adjust for your vehicle
    max_longitudinal_decel: -10.0  # Adjust for your vehicle  
    max_lateral_accel: 12.0        # Adjust for your vehicle
    default_friction_coefficient: 1.0
```

### 3. Gradual Tuning Process:

```yaml
# Week 1: Very conservative (safe but slow)
max_longitudinal_accel: 3.0
max_longitudinal_decel: -5.0
max_lateral_accel: 6.0

# Week 2: Slightly more aggressive  
max_longitudinal_accel: 4.0
max_longitudinal_decel: -7.0
max_lateral_accel: 8.0

# Week 3: Approach realistic limits
max_longitudinal_accel: 5.0
max_longitudinal_decel: -9.0
max_lateral_accel: 10.0

# Continue until you find the sweet spot
```

## Validation and Safety

### Safety Considerations:
- **Always start conservative** - you can increase limits later
- **Test in safe environments** only
- **Have safety systems active** (emergency stops, etc.)
- **Monitor vehicle behavior** closely during initial testing

### Validation Methods:
1. **Visual Inspection**: Do the planned trajectories look reasonable?
2. **Execution Testing**: Can the vehicle actually follow the trajectories?
3. **Performance Comparison**: How does lap time compare to manual driving?
4. **Stability Assessment**: Is the vehicle stable during trajectory execution?

### Warning Signs:
- **Trajectories too aggressive**: Vehicle slides, loses control
- **Trajectories too conservative**: Much slower than manual driving
- **Oscillatory behavior**: Vehicle weaves or oscillates
- **Unrealistic maneuvers**: Impossible cornering speeds

## Expected Performance

With properly tuned parameters, you should expect:
- **Lap times**: 10-20% slower than optimal (with real GGGV data)
- **Stability**: Good stability with conservative margins
- **Consistency**: Repeatable performance
- **Safety**: Safe operation within limits

## Getting Real GGGV Data Later

When you're ready to get real GGGV data:

1. **Professional Testing**: Hire a vehicle dynamics engineer
2. **Simulation**: Use detailed vehicle simulation software
3. **Incremental Testing**: Gradually expand the envelope through testing
4. **Data Logging**: Record accelerations during normal operation

## Troubleshooting

### Problem: Trajectories are too slow
**Solution**: Gradually increase acceleration limits by 10-20%

### Problem: Vehicle is unstable
**Solution**: Decrease limits by 20-30%, check tire pressure, alignment

### Problem: Unrealistic cornering speeds
**Solution**: Reduce `max_lateral_accel`, check track curvature data

### Problem: Poor braking performance  
**Solution**: Increase `max_longitudinal_decel` (more negative), check brake system

## Example Complete Configuration

```yaml
# Complete example for a high-performance go-kart
tam_sampling_planner:
  gggv:
    max_longitudinal_accel: 5.5
    max_longitudinal_decel: -8.5
    max_lateral_accel: 11.0
    
    # Machine limits (engine power curve)
    ax_machine_limits: [5.5, 5.2, 4.8, 4.4, 4.0, 3.6, 3.2, 2.8, 2.4, 2.0]
    
    # Velocity scaling
    velocity_breakpoints: [0, 5, 10, 15, 20, 25, 30, 35, 40, 45]
    accel_scale_factors: [0.9, 1.0, 1.0, 0.98, 0.95, 0.90, 0.85, 0.8, 0.7, 0.6]
    
    # Grip
    default_friction_coefficient: 1.1  # Good racing tires
    
    # Shape parameters
    gg_exponent_ax_pos: 2.0
    gg_exponent_ax_neg: 2.0
```

Remember: **Start conservative, tune gradually, prioritize safety!**
