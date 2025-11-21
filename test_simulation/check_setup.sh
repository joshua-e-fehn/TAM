#!/bin/bash
# Quick setup verification script for racing test framework

echo "=========================================="
echo "🔍 Racing Test Framework Setup Check"
echo "=========================================="
echo

# Check if configuration file exists
echo "1. Configuration File:"
if [ -f "race_test_config.yaml" ]; then
    echo "   ✅ race_test_config.yaml found"
    num_tests=$(grep -c "simulation_id:" race_test_config.yaml)
    echo "   📊 Contains $num_tests test configurations"
else
    echo "   ❌ race_test_config.yaml NOT FOUND"
fi
echo

# Check if framework script exists
echo "2. Framework Script:"
if [ -f "race_test_framework.py" ]; then
    echo "   ✅ race_test_framework.py found"
    if [ -x "race_test_framework.py" ]; then
        echo "   ✅ Script is executable"
    else
        echo "   ⚠️  Script is not executable (run: chmod +x race_test_framework.py)"
    fi
else
    echo "   ❌ race_test_framework.py NOT FOUND"
fi
echo

# Check if roscore is running
echo "3. ROS Environment:"
if pgrep -x "roscore" > /dev/null || pgrep -x "rosmaster" > /dev/null; then
    echo "   ✅ roscore is running"
else
    echo "   ⚠️  roscore is NOT running"
    echo "      Start with: roscore"
fi
echo

# Check if multi_car.launch exists
echo "4. Launch File:"
LAUNCH_FILE="$HOME/catkin_ws/src/race_stack/stack_master/launch/multi_car.launch"
if [ -f "$LAUNCH_FILE" ]; then
    echo "   ✅ multi_car.launch found"
else
    echo "   ❌ multi_car.launch NOT FOUND at expected location"
    echo "      Expected: $LAUNCH_FILE"
fi
echo

# Check Python dependencies
echo "5. Python Dependencies:"
python3 -c "import rospy; import yaml" 2>/dev/null
if [ $? -eq 0 ]; then
    echo "   ✅ Required Python packages available (rospy, yaml)"
else
    echo "   ❌ Missing Python packages"
    echo "      Install with: pip3 install pyyaml"
fi
echo

echo "=========================================="
echo "Setup check complete!"
echo "=========================================="
