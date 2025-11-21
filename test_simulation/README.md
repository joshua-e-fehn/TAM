# Racing Simulation Test Framework

Simple framework for running automated racing tests with multiple cars and different planner configurations.

## Overview

This framework (`Rahmenprogramm`) launches multiple race simulations sequentially with different parameters. Each simulation runs until the `/simulation_complete` parameter is set to `True` by the race event monitor.

## Features

- ✅ **Sequential Test Execution**: Run multiple tests one after another
- ✅ **Parameter Matrix**: Configure different planners, speeds, and accelerations for each car
- ✅ **Simulation ID Tracking**: Each test gets a unique ID for logging and analysis
- ✅ **Overtaking Sector Management**: Automatically checks and enables overtaking sectors
- ✅ **Automatic Race Start**: Waits for cars to be READY and starts race via service
- ✅ **Race Event Monitoring**: Integrates with `race_event_monitor.py` for automatic completion detection
- ✅ **Clean Simulation Lifecycle**: Proper startup, monitoring, and shutdown for each test

## Quick Start

### 1. Setup

Make sure roscore is running:
```bash
roscore
```

Verify your setup:
```bash
cd /home/atlas/catkin_ws/testSimulation
./check_setup.sh
```

### 2. Configure Tests

Edit `race_test_config.yaml` to define your test matrix:

```yaml
test_matrix:
  - simulation_id: 1
    name: "test_1_baseline"
    planner_car1: "predictive_spliner"
    planner_car2: "predictive_spliner"
    speed_multiplier_car1: 1.0
    speed_multiplier_car2: 1.0
    accel_multiplier_car1: 1.0
    accel_multiplier_car2: 1.0
    global_map: "f"
```

### 3. Run Tests

```bash
python3 race_test_framework.py
```

The framework will:
1. Launch each simulation with specified parameters
2. Set the simulation ID
3. Check and enable overtaking sectors
4. Wait for cars to reach READY state
5. Start the race automatically
6. Monitor for race completion (via `/simulation_complete` parameter)
7. Terminate the simulation cleanly
8. Move to the next test

### 4. Check Results

Race events are logged to `/tmp/race_logs/` by the race event monitor.

Check current parameters during a test:
```bash
./check_params.sh
```

## Test Configuration

Each test in the matrix supports:

- **simulation_id**: Unique identifier for the test
- **name**: Descriptive test name
- **planner_car1/car2**: Planner to use (`predictive_spliner`, `tam_sampling`, etc.)
- **speed_multiplier_car1/car2**: Speed scaling factor (0.0-1.0+)
- **accel_multiplier_car1/car2**: Acceleration scaling factor (0.0-1.0+)
- **global_map**: Map to use (e.g., "f")

## Race Completion Conditions

The framework monitors `/simulation_complete` which is set by `race_event_monitor.py` when any of these occur:

1. **Lap Completion**: Car2 finishes target number of laps
2. **Collision**: Cars collide (distance ≤ 0.5m)
3. **Track Boundary**: Car goes off track (|d-coordinate| > 1.5m)
4. **Overtake Lead**: Car1 overtakes and leads by 10m+

## Helper Scripts

- **check_setup.sh**: Verify framework installation and dependencies
- **check_params.sh**: Display current simulation parameters and overtaking sectors
- **test_ot_check.py**: Test overtaking sector checking in isolation

## Integration with Race Event Monitor

This framework works with `race_event_monitor.py` (in `multi_car_interaction` package):

- Framework launches simulation and sets `/simulation_id`
- Race event monitor logs events to CSV with simulation ID
- Monitor sets `/simulation_complete=True` when race ends
- Framework detects completion and terminates simulation

## Troubleshooting

### Tests not starting
- Check if roscore is running: `pgrep roscore`
- Verify launch file exists: `ls -la ~/catkin_ws/src/race_stack/stack_master/launch/multi_car.launch`

### Overtaking sectors not enabled
- Run test script: `python3 test_ot_check.py`
- Manually check: `rosparam get /ot_map_params`

### Simulation hangs
- Default timeout is 300s (5 minutes)
- Check race event monitor is running: `rosnode list | grep race_event_monitor`
- Manually set completion: `rosparam set /simulation_complete true`

### Race doesn't start automatically
- Check if race_control service exists: `rosservice list | grep race_control`
- Verify cars reach READY state: `rosparam get /car1/state_machine/current_state`

## File Structure

```
testSimulation/
├── race_test_framework.py      # Main test framework
├── race_test_config.yaml        # Test parameter matrix
├── check_setup.sh               # Setup verification
├── check_params.sh              # Parameter checking
├── test_ot_check.py            # Overtaking sector test
└── README.md                    # This file
```

## Example Output

```
======================================================================
🏎️  Racing Simulation Test Framework
======================================================================
Loaded 3 test configurations

======================================================================
🏁 Test 1/3: test_1_baseline (ID: 1)
======================================================================
   Car 1: predictive_spliner (speed: 1.0x, accel: 1.0x)
   Car 2: predictive_spliner (speed: 1.0x, accel: 1.0x)
   Map: f

🚀 Launching simulation...
⏳ Waiting for simulation to initialize (15 seconds)...
✅ Set /simulation_id to 1
🔍 Checking overtaking sectors...
   [Global] Found 5 overtaking sectors
      ✅ Sector 0: already enabled
      ...
⏳ Waiting for cars to be READY...
   ✅ Both cars are READY!
🏁 Starting race via service call...
   ✅ Race started successfully!
⏱️  Monitoring simulation...
✅ Simulation complete after 45.3s
   Reason: lap_completion

🛑 Terminating simulation...
✅ Simulation terminated cleanly
```

## Advanced Usage

### Modifying Timeout

Edit `race_test_framework.py` and change the timeout in `wait_for_completion()`:

```python
result = self.wait_for_completion(timeout=600)  # 10 minutes
```

### Adding More Tests

Simply add more entries to `race_test_config.yaml`:

```yaml
test_matrix:
  - simulation_id: 4
    name: "test_4_custom"
    # ... configuration
```

### Running Specific Tests

You can modify the framework to accept command-line arguments for running specific test IDs.

## See Also

- `UPDATES.md` - Feature changelog
- `/tmp/race_logs/` - Race event logs
- `race_event_monitor.py` - Race completion detection node
