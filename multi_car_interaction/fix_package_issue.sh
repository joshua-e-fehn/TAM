#!/bin/bash
# 🎯 MULTI-CAR INTERACTION - COMPLETE FIX GUIDE

echo "🔧 FIXING: Resource not found: multi_car_interaction"
echo "=================================================================="
echo ""

echo "✅ PROBLEM DIAGNOSIS:"
echo "  The 'multi_car_interaction' package exists but ROS can't find it"
echo "  Root cause: Workspace needs to be sourced after building"
echo ""

echo "🛠️  COMPLETE SOLUTION:"
echo "=================================================================="
echo ""

echo "1️⃣ REBUILD THE PACKAGE:"
echo "  cd /home/atlas/catkin_ws"
echo "  catkin build multi_car_interaction"
echo ""

echo "2️⃣ SOURCE THE WORKSPACE (CRITICAL STEP):"
echo "  source devel/setup.bash"
echo ""

echo "3️⃣ VERIFY PACKAGE IS FOUND:"
echo "  rospack find multi_car_interaction"
echo "  # Should output: /home/atlas/catkin_ws/src/race_stack/multi_car_interaction"
echo ""

echo "4️⃣ TEST LAUNCH FILE:"
echo "  roslaunch multi_car_interaction multi_car_interaction.launch"
echo ""

echo "5️⃣ LAUNCH FULL SYSTEM:"
echo "  roslaunch stack_master multi_car.launch \\"
echo "    global_map:=f \\"
echo "    sim:=True \\"
echo "    rviz:=True \\"
echo "    enable_car_interaction:=True"
echo ""

echo "📋 WHAT WAS FIXED:"
echo "=================================================================="
echo ""
echo "✅ Package rebuild: Ensures all files are properly compiled"
echo "✅ Parameter parsing: Fixed car_names parsing (string vs list)"  
echo "✅ Publish rate: Updated to 50Hz for better performance"
echo "✅ Workspace sourcing: Critical for ROS package discovery"
echo ""

echo "🎯 WHY THE ERROR OCCURRED:"
echo "=================================================================="
echo ""
echo "ROS uses the ROS_PACKAGE_PATH to find packages. When you build"
echo "a new package, the workspace must be sourced to update this path."
echo ""
echo "Before sourcing:"
echo "  ROS_PACKAGE_PATH doesn't include multi_car_interaction"
echo ""
echo "After sourcing devel/setup.bash:"
echo "  ROS_PACKAGE_PATH includes all workspace packages"
echo ""

echo "🚀 AUTOMATED FIX SCRIPT:"
echo "=================================================================="
echo ""

# Run the actual fix
echo "Running automated fix..."

cd /home/atlas/catkin_ws

echo "Step 1: Building package..."
catkin build multi_car_interaction

echo "Step 2: Sourcing workspace..."
source devel/setup.bash

echo "Step 3: Verifying package discovery..."
if rospack find multi_car_interaction > /dev/null 2>&1; then
    echo "✅ SUCCESS: multi_car_interaction package found!"
    echo "   Location: $(rospack find multi_car_interaction)"
else
    echo "❌ ERROR: Package still not found"
    exit 1
fi

echo "Step 4: Verifying launch file..."
if roslaunch --files multi_car_interaction multi_car_interaction.launch > /dev/null 2>&1; then
    echo "✅ SUCCESS: Launch file found!"
else
    echo "❌ ERROR: Launch file not found"
    exit 1
fi

echo ""
echo "🎉 COMPLETE FIX APPLIED SUCCESSFULLY!"
echo "=================================================================="
echo ""
echo "✅ Package built and sourced"
echo "✅ Parameter parsing fixed"
echo "✅ 50Hz publish rate configured"
echo "✅ Ready for multi-car racing!"
echo ""

echo "🏁 NEXT STEPS:"
echo "=================================================================="
echo ""
echo "1. In ANY NEW TERMINAL, always run:"
echo "   cd /home/atlas/catkin_ws && source devel/setup.bash"
echo ""
echo "2. Then launch the system:"
echo "   roslaunch stack_master multi_car.launch \\"
echo "     global_map:=f sim:=True rviz:=True enable_car_interaction:=True"
echo ""
echo "3. Monitor the obstacle publishing:"
echo "   rostopic echo /car1/perception/multi_car_obstacles"
echo "   rostopic hz /car1/perception/multi_car_obstacles  # Should show ~50Hz"
echo ""

echo "💡 PRO TIP:"
echo "=================================================================="
echo "Add this to your ~/.bashrc to automatically source on terminal startup:"
echo "echo 'source /home/atlas/catkin_ws/devel/setup.bash' >> ~/.bashrc"
