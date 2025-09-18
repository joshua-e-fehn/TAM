#!/bin/bash
# Race Control Commands
# Run this script to see available race control commands

echo "=================================="
echo "🏁 RACE CONTROL COMMANDS"
echo "=================================="
echo ""
echo "Start individual cars:"
echo "  rosservice call /race_control/start_car1"
echo "  rosservice call /race_control/start_car2"
echo ""
echo "Start both cars simultaneously:"
echo "  rosservice call /race_control/start_both"
echo ""
echo "Reset cars to READY state:"
echo "  rosservice call /race_control/reset_cars"
echo ""
echo "Emergency stop:"
echo "  rosservice call /race_control/emergency_stop"
echo ""
echo "Monitor car states:"
echo "  rostopic echo /car1/state_machine"
echo "  rostopic echo /car2/state_machine"
echo ""
echo "=================================="
echo ""

# Check if services are available
echo "Checking service availability..."
if rosservice list | grep -q "/race_control/"; then
    echo "✅ Race control services are available!"
    echo ""
    echo "Available services:"
    rosservice list | grep "/race_control/"
else
    echo "❌ Race control services not found."
    echo "Make sure multi_car simulation is running with enable_race_start_controller:=True"
fi
echo ""
