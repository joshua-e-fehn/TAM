# Map Parsing - Documentation

## Overview

The TAM to ETH Map Parser converts TAM/Marina raceline CSV data into the F1Tenth race stack map format. It generates:

- `global_waypoints.json` - Trajectory data for all racing lines
- Track images (`.png`) - Visual representation of the track
- Configuration files (`.yaml`) - Map, speed scaling, sectors
- Optimized racing lines using trajectory optimization

---

## Basic Usage

```bash
cd /home/atlas/catkin_ws

# Basic conversion with defaults
python3 src/race_stack/tam_to_eth_map_parser/map_parser/basic_tam_to_eth_map_parser.py \
    src/race_stack/tam/maps/marina.csv

# Custom output name and scale
python3 src/race_stack/tam_to_eth_map_parser/map_parser/basic_tam_to_eth_map_parser.py \
    src/race_stack/tam/maps/marina.csv \
    --output-name my_map \
    --scale-factor 0.12 \
    --car-name NUC2

# Full example with all options
python3 src/race_stack/tam_to_eth_map_parser/map_parser/basic_tam_to_eth_map_parser.py \
    src/race_stack/tam/maps/marina.csv \
    --output-name my_map \
    --scale-factor 0.20 \
    --width-multiplier 1.0 \
    --car-name NUC2 \
    --racing-line-type mintime
```

---

## Command Line Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `csv_file` | (required) | Path to TAM CSV file |
| `--output-name` | `marina` | Base name for output map directory |
| `--scale-factor` | `0.1` | Scale factor for map size (0.1 = 10% of original) |
| `--width-multiplier` | `1.0` | Track width multiplier (1.0 = preserve original) |
| `--car-name` | `NUC2` | Car configuration name for trajectory optimization |
| `--racing-line-type` | `mintime` | Racing line optimization: `mintime`, `mincurv`, `disable` |

---

## Output Map Naming

The output map name is automatically generated based on parameters:

```
{output-name}_{scale%}s_{width%}w_{car-name}_{racing-line-type}
```

### Examples

| Parameters | Generated Name |
|------------|----------------|
| `--output-name my_map --scale-factor 0.20 --car-name NUC2` | `my_map_20%s_100%w_NUC2_mintime` |
| `--output-name marina --scale-factor 0.12` | `marina_12%s_100%w_NUC2_mintime` |
| `--output-name test --scale-factor 0.1 --width-multiplier 0.8` | `test_10%s_80%w_NUC2_mintime` |

---

## Scale Factor Guidelines

The scale factor reduces the physical size of the track:

| Scale Factor | Result | Use Case |
|--------------|--------|----------|
| `1.0` | Full size (100%) | Physical deployment |
| `0.2` | 20% of original | Large simulation tests |
| `0.12` | 12% of original | Medium simulation |
| `0.1` | 10% of original | Fast testing (default) |
| `0.05` | 5% of original | Very small tracks |

### Considerations

- **Smaller scale** = faster simulations, but may have unrealistic dynamics
- **Larger scale** = more realistic, but slower simulations
- **Width multiplier** affects track width independently from scale

---

## Racing Line Types

| Type | Description |
|------|-------------|
| `mintime` | Minimum time optimization (fastest lap) |
| `mincurv` | Minimum curvature (smoothest path) |
| `disable` | Skip racing line optimization (use centerline) |

---

## Generated Files

The parser creates a complete map directory in `stack_master/maps/`:

```
stack_master/maps/{map_name}/
├── global_waypoints.json     # All trajectory data
├── {map_name}.png           # Track image
├── {map_name}.yaml          # Map configuration
├── speed_scaling.yaml       # Speed scaling sectors
├── ot_sectors.yaml          # Overtaking sectors
├── starting_position.yaml   # Car starting positions
└── cache/                   # Optimization cache files
```

### global_waypoints.json Structure

```json
{
  "centerline": {
    "s_m": [...],     // Arc length
    "x_m": [...],     // X coordinates
    "y_m": [...],     // Y coordinates
    "psi_rad": [...], // Heading angles
    "vx_mps": [...],  // Velocities
    "d_left": [...],  // Distance to left boundary
    "d_right": [...]  // Distance to right boundary
  },
  "min_curv": { ... },
  "shortest_path": { ... },
  "min_time": { ... }
}
```

---

## Input CSV Format

The parser expects TAM/Marina format CSV with these columns:

| Column | Description |
|--------|-------------|
| `rl_x_m`, `rl_y_m` | Racing line coordinates |
| `rl_vx_mps` | Racing line velocity |
| `rl_psi_rad` | Racing line heading |
| `ref_cl_x_m`, `ref_cl_y_m` | Centerline coordinates |
| `ref_cl_psi_rad` | Centerline heading |
| `ref_cl_d_left`, `ref_cl_d_right` | Track widths |
| `tb_left_x`, `tb_left_y` | Left track boundary |
| `tb_right_x`, `tb_right_y` | Right track boundary |

---

## Example Workflows

### Creating a New Map for Testing

```bash
# Step 1: Parse the map
python3 src/race_stack/tam_to_eth_map_parser/map_parser/basic_tam_to_eth_map_parser.py \
    src/race_stack/tam/maps/marina.csv \
    --output-name marina \
    --scale-factor 0.12 \
    --car-name NUC2

# Step 2: Verify map was created
ls src/race_stack/stack_master/maps/marina_12%s_100%w_NUC2_mintime/

# Step 3: Launch simulation with new map
roslaunch stack_master single_car.launch \
    map_name:=marina_12%s_100%w_NUC2_mintime \
    planner:=predictive_spliner
```

### Creating Multiple Scale Variants

```bash
# Small scale for fast testing
python3 src/race_stack/tam_to_eth_map_parser/map_parser/basic_tam_to_eth_map_parser.py \
    src/race_stack/tam/maps/marina.csv \
    --output-name marina --scale-factor 0.10

# Medium scale for detailed testing
python3 src/race_stack/tam_to_eth_map_parser/map_parser/basic_tam_to_eth_map_parser.py \
    src/race_stack/tam/maps/marina.csv \
    --output-name marina --scale-factor 0.20

# Large scale for realistic simulation
python3 src/race_stack/tam_to_eth_map_parser/map_parser/basic_tam_to_eth_map_parser.py \
    src/race_stack/tam/maps/marina.csv \
    --output-name marina --scale-factor 0.50
```

### Wider Track for Easier Overtaking

```bash
# 150% track width
python3 src/race_stack/tam_to_eth_map_parser/map_parser/basic_tam_to_eth_map_parser.py \
    src/race_stack/tam/maps/marina.csv \
    --output-name marina \
    --scale-factor 0.12 \
    --width-multiplier 1.5
```

---

## Key Features

### Clean Scaling Separation

All input data is scaled upfront before any processing:
- Coordinates (x, y) are scaled
- Distances (s, d_left, d_right) are scaled
- Velocities are scaled proportionally
- Curvatures are inverse-scaled
- Track is translated to origin (0, 0)

### Trajectory Optimization

The parser uses the global trajectory optimizer to generate:
- Minimum curvature racing line
- Shortest path
- Minimum time racing line

### Velocity Optimization

If the input has constant velocity, the parser applies velocity optimization based on:
- Track curvature limits
- Vehicle dynamics (car configuration)
- Friction constraints

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| CSV not found | Check path is relative to current directory |
| Missing columns | Ensure CSV matches expected TAM format |
| Optimization fails | Try `--racing-line-type disable` to skip optimization |
| Map not loading | Check map name matches exactly (including special chars) |
| Track too small | Increase `--scale-factor` |

### Debug Output

The parser provides detailed output during processing:

```
Using scale factor: 0.12 (map will be 12.0% of original size)
Using width multiplier: 1.0 (track will be 100% of original width)
Generated map name: marina_12%s_100%w_NUC2_mintime
Using car configuration: NUC2 for trajectory optimization

============================================================
📂 Step 1: Loading and Scaling CSV Data
============================================================
📄 Loading TAM CSV: src/race_stack/tam/maps/marina.csv
✅ Found 1234 data lines

📊 Parsed raw waypoints:
  🔵 Centerline: 1234 waypoints
  🔴 IQP (racing line): 1234 waypoints
  🟡 Trackbounds: 1234 left, 1234 right
```

---

## Using Generated Maps

### Single Car Mode

```bash
roslaunch stack_master single_car.launch \
    map_name:=marina_12%s_100%w_NUC2_mintime
```

### Multi-Car Mode

```bash
roslaunch stack_master multi_car.launch \
    global_map:=marina_12%s_100%w_NUC2_mintime
```

### Test Framework

```yaml
# In test config file
test_matrix:
  - simulation_id: "test_001"
    global_map: "marina_12%s_100%w_NUC2_mintime"
    # ...
```
