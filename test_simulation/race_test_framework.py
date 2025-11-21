#!/usr/bin/env python3
"""
Simple Racing Simulation Test Framework (Rahmenprogramm)

Launches multiple race simulations sequentially with different parameters.
Each simulation runs until /simulation_complete parameter is True.

Usage:
    python3 race_test_framework.py
"""

import rospy
import subprocess
import time
import yaml
import os
import signal
import sys
import argparse
from std_srvs.srv import Empty, EmptyResponse


class RaceTestFramework:
    def __init__(self, mode='multi_car', config_file='race_test_run_config.yaml', default_config_file='race_test_default_config.yaml', rviz='true'):
        """Initialize the test framework

        Args:
            mode: Test mode - 'single_car_no_obstacle', 'single_car_obstacle', 'multi_car', or 'all'
            config_file: Path to test configuration YAML file
            default_config_file: Path to default parameters YAML file
            rviz: Enable RViz visualization ('true' or 'false')
        """

        self.mode = mode
        self.rviz = rviz
        self.test_configs = []  # Will hold all test configurations to run
        # Track loaded config file paths for saving to batch dir
        self.loaded_config_files = []

        # Generate batch number based on timestamp (as string to avoid XML-RPC int limits)
        from datetime import datetime
        self.batch_number = datetime.now().strftime("%Y%m%d%H%M%S")

        print("="*70)
        print("🏎️  Racing Simulation Test Framework")
        print("="*70)
        print(f"Batch Number: {self.batch_number}")
        print(f"Test Mode: {mode}")
        print()

        # Set batch number as ROS parameter immediately (before any nodes launch)
        try:
            import rospy
            rospy.set_param('/race_test/batch_number', self.batch_number)
        except:
            pass  # ROS not initialized yet, will be set later

        # Load configurations based on mode
        if mode == 'all':
            # Load all three config files in sequence
            self._load_all_configs()
        else:
            # Load single config file
            self._load_single_config(config_file)

        # Load default configuration
        self.default_config_path = os.path.join(
            os.path.dirname(__file__), default_config_file)
        self.default_config = None

        if os.path.exists(self.default_config_path):
            with open(self.default_config_path, 'r') as f:
                self.default_config = yaml.safe_load(f)
            print(f"📄 Loaded default configuration from {default_config_file}")
        else:
            print(
                f"⚠️  Default configuration file not found: {self.default_config_path}")

        self.launch_process = None
        self.test_results = []

        print(f"Loaded {len(self.test_configs)} total test configurations")
        print()

        # Default predictive spliner switch (can be overridden per test)
        self.default_use_global_prediction = True

    def _load_single_config(self, config_file):
        """Load a single configuration file"""
        config_path = os.path.join(os.path.dirname(__file__), config_file)

        if not os.path.exists(config_path):
            print(f"❌ Configuration file not found: {config_path}")
            sys.exit(1)

        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        # Add all tests from this config
        self.test_configs.extend(config['test_matrix'])
        # Track the config file path for later saving
        self.loaded_config_files.append(config_path)
        print(
            f"📄 Loaded {len(config['test_matrix'])} tests from {config_file}")

    def _load_all_configs(self):
        """Load all three config files for 'all' mode"""
        config_files = [
            ('single_car_no_obstacle_config.yaml', 'Single Car (No Obstacle)'),
            ('single_car_with_obstacle_config.yaml', 'Single Car (With Obstacle)'),
            ('multi_car_config.yaml', 'Multi-Car')
        ]

        print("📦 Loading all configurations...")
        for config_file, description in config_files:
            config_path = os.path.join(os.path.dirname(__file__), config_file)

            if not os.path.exists(config_path):
                print(f"⚠️  Skipping {description}: {config_file} not found")
                continue

            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)

            num_tests = len(config['test_matrix'])
            self.test_configs.extend(config['test_matrix'])
            # Track the config file path for later saving
            self.loaded_config_files.append(config_path)
            print(f"   ✅ {description}: {num_tests} tests from {config_file}")

    def save_config_files_to_batch_dir(self):
        """Save test configuration files to batch log directory for cross-reference"""
        # Base log directory
        base_log_dir = os.path.join(os.path.expanduser(
            '~'), 'catkin_ws', 'testSimulation', 'logs')

        # Create mode-specific subdirectory
        mode_dir = os.path.join(base_log_dir, self.mode)

        # Create batch-specific subdirectory
        batch_dir = os.path.join(mode_dir, f"batch_{self.batch_number}")

        # Create batch directory if it doesn't exist
        os.makedirs(batch_dir, exist_ok=True)

        print(f"📋 Saving test configuration files to batch directory...")

        # Save each loaded config file
        for config_path in self.loaded_config_files:
            try:
                # Get the base filename
                config_filename = os.path.basename(config_path)
                dest_path = os.path.join(batch_dir, config_filename)

                # Copy the file
                import shutil
                shutil.copy2(config_path, dest_path)
                print(f"   ✅ Saved {config_filename} to {batch_dir}")
            except Exception as e:
                print(f"   ⚠️  Failed to save {config_filename}: {e}")

        # Also save the default config if it exists
        if self.default_config_path and os.path.exists(self.default_config_path):
            try:
                default_filename = os.path.basename(self.default_config_path)
                dest_path = os.path.join(batch_dir, default_filename)
                import shutil
                shutil.copy2(self.default_config_path, dest_path)
                print(f"   ✅ Saved {default_filename} to {batch_dir}")
            except Exception as e:
                print(f"   ⚠️  Failed to save default config: {e}")

        print(f"   📁 Batch log directory: {batch_dir}")
        return batch_dir

    def terminate_existing_simulations(self):
        """Terminate any existing simulations that may be running"""
        print("   🔍 Looking for running simulations...")

        # Kill Gazebo (main simulation components)
        result1 = subprocess.run(['pkill', '-9', 'gzserver'],
                                 stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)
        result2 = subprocess.run(['pkill', '-9', 'gzclient'],
                                 stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)

        # Kill RViz
        subprocess.run(['pkill', '-9', 'rviz'],
                       stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL)

        # Kill roslaunch processes
        subprocess.run(['pkill', '-9', 'roslaunch'],
                       stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL)

        # Kill planners and controllers
        subprocess.run(['pkill', '-9', '-f', 'predictive_spliner'],
                       stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL)
        subprocess.run(['pkill', '-9', '-f', 'tam_sampling'],
                       stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL)
        subprocess.run(['pkill', '-9', '-f', 'race_event_monitor'],
                       stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL)
        subprocess.run(['pkill', '-9', '-f', 'race_start_controller'],
                       stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL)

        # Check if anything was killed
        if result1.returncode == 0 or result2.returncode == 0:
            print("   ✅ Terminated existing simulation")
            time.sleep(2)  # Wait for cleanup
        else:
            print("   ℹ️  No existing simulation found")

    def kill_all_ros_nodes(self):
        """Kill all ROS nodes and processes for complete cleanup"""
        print("🧹 Cleaning up all ROS processes...")

        try:
            # Kill all ROS-related processes in the correct order
            # First kill all simulation nodes (gazebo, rviz, etc.)
            ros_processes = ['gzserver', 'gzclient', 'rviz', 'roslaunch']
            for proc in ros_processes:
                subprocess.run(['pkill', '-9', proc],
                               stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL)

            # Kill all Python ROS nodes (planners, controllers, perception, etc.)
            python_nodes = [
                'controller_manager', 'tracking.py', 'state_machine_node',
                'predictive_spliner', 'tam_sampling', 'race_event_monitor',
                'race_start_controller', 'frenet_converter'
            ]
            for proc in python_nodes:
                subprocess.run(['pkill', '-9', '-f', proc],
                               stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL)

            # Kill any remaining Python ROS nodes
            subprocess.run(['pkill', '-9', '-f', '/car[12]/'],
                           stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL)

            time.sleep(8)  # Wait longer for all nodes to die

            # Then kill ROS core components
            core_processes = ['rosout', 'rosmaster', 'roscore']
            for proc in core_processes:
                subprocess.run(['pkill', '-9', proc],
                               stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL)

            print("   ✅ Killed all ROS processes")
            time.sleep(2)  # Wait longer for all processes to fully terminate
            return True

        except Exception as e:
            print(f"   ⚠️  Error during cleanup: {e}")
            return False

    def restart_roscore(self):
        """Restart roscore for a clean ROS environment"""
        print("🔄 Restarting roscore...")

        try:
            # First ensure everything is killed
            self.kill_all_ros_nodes()

            # Wait a bit to ensure all old processes are fully cleaned up
            time.sleep(8)

            # Start new roscore in background
            subprocess.Popen(['roscore'],
                             stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL,
                             preexec_fn=os.setsid)

            print("   ✅ Started new roscore")
            time.sleep(3)  # Wait longer for roscore to fully initialize
            return True

        except Exception as e:
            print(f"   ⚠️  Error restarting roscore: {e}")
            return False

    def launch_simulation(self, test_config):
        """Launch roslaunch with specific test parameters"""

        # Build roslaunch command
        cmd = [
            'roslaunch', 'stack_master', 'multi_car.launch',
            f'planner_car1:={test_config["planner_car1"]}',
            f'planner_car2:={test_config["planner_car2"]}',
            f'global_map:={test_config["global_map"]}',
            f'speed_multiplier_car1:={test_config["speed_multiplier_car1"]}',
            f'speed_multiplier_car2:={test_config["speed_multiplier_car2"]}',
            f'accel_multiplier_car1:={test_config["accel_multiplier_car1"]}',
            f'accel_multiplier_car2:={test_config["accel_multiplier_car2"]}',
            f'global_map:={test_config["global_map"]}',
            f'rviz:={self.rviz}',
            'enable_race_start_controller:=true'
        ]

        print(f"🚀 Launching simulation...")
        print(f"   Command: {' '.join(cmd)}")

        # Determine if we should show simulation output
        show_output = test_config.get('show_simulation_output', False)

        if show_output:
            # Launch with output visible in terminal
            print("   📺 Simulation output will be shown below:")
            print("   " + "="*66)
            self.launch_process = subprocess.Popen(
                cmd,
                preexec_fn=os.setsid  # Create new process group for clean termination
            )
        else:
            # Launch with output suppressed (redirect to /dev/null to prevent blocking)
            # Using PIPE can cause processes to block when buffers fill up
            devnull = open(os.devnull, 'w')
            self.launch_process = subprocess.Popen(
                cmd,
                stdout=devnull,
                stderr=devnull,
                preexec_fn=os.setsid  # Create new process group for clean termination
            )

        print("⏳ Waiting for simulation to initialize (15 seconds)...")
        time.sleep(15)

        return True

    def launch_single_car_simulation(self, test_config):
        """Launch single_car.launch with specific test parameters"""

        # Determine if dummy obstacle is enabled based on mode
        enable_obstacle = (test_config.get('mode') == 'single_car_obstacle')

        # Build roslaunch command for single_car.launch
        cmd = [
            'roslaunch', 'stack_master', 'single_car.launch',
            f'planner:={test_config["planner"]}',
            f'map_name:={test_config["global_map"]}',
            f'speed_multiplier:={test_config.get("speed_multiplier", 1.0)}',
            f'accel_multiplier:={test_config.get("accel_multiplier", 1.0)}',
            f'enable_dummy_obstacle:={str(enable_obstacle).lower()}',
            f'rviz:={self.rviz}',
            'enable_race_start_controller:=true'
        ]

        # Add obstacle-specific parameters if enabled
        if enable_obstacle:
            # Set map-specific default speed limits if not specified
            default_speed_limit = 10.0  # Generic default
            if test_config.get('global_map') == 'f':
                default_speed_limit = 6.35  # F-track actual max speed

            cmd.extend([
                f'obstacle_trajectory:={test_config.get("obstacle_trajectory", "min_curv")}',
                f'obstacle_start_s:={test_config.get("obstacle_start_s", 0)}',
                f'obstacle_speed:={test_config.get("obstacle_speed", 0.5)}',
                f'obstacle_constant_speed:={str(test_config.get("obstacle_constant_speed", False)).lower()}',
                f'obstacle_path_amplitude:={test_config.get("obstacle_path_amplitude", 0.0)}',
                f'obstacle_path_frequency:={test_config.get("obstacle_path_frequency", 0.15)}',
                f'obstacle_path_phase:={test_config.get("obstacle_path_phase", 0.0)}',
                f'obstacle_max_speed_limit:={test_config.get("obstacle_max_speed_limit", default_speed_limit)}',
                f'obstacle_max_accel:={test_config.get("obstacle_max_accel", 3.0)}'
            ])

        print(f"🚀 Launching single-car simulation...")
        print(
            f"   Obstacle mode: {'ENABLED' if enable_obstacle else 'DISABLED'}")
        print(f"   Command: {' '.join(cmd)}")

        # Determine if we should show simulation output
        show_output = test_config.get('show_simulation_output', False)

        if show_output:
            # Launch with output visible in terminal
            print("   📺 Simulation output will be shown below:")
            print("   " + "="*66)
            self.launch_process = subprocess.Popen(
                cmd,
                preexec_fn=os.setsid
            )
        else:
            # Launch with output suppressed (redirect to /dev/null to prevent blocking)
            # Using PIPE can cause processes to block when buffers fill up
            devnull = open(os.devnull, 'w')
            self.launch_process = subprocess.Popen(
                cmd,
                stdout=devnull,
                stderr=devnull,
                preexec_fn=os.setsid
            )

        print("⏳ Waiting for simulation to initialize (15 seconds)...")
        time.sleep(15)

        return True

    def launch_race_event_monitor(self, test_config):
        """Launch race_event_monitor with appropriate mode"""

        mode = test_config.get('mode', 'multi_car')

        # Determine race_mode parameter for monitor
        if mode == 'single_car_no_obstacle':
            race_mode = 'single_car_no_obstacle'
        elif mode == 'single_car_obstacle':
            race_mode = 'single_car_obstacle'
        else:
            # Multi-car mode - monitor is launched by multi_car.launch
            return True

        # Launch single-car event monitor
        cmd = [
            'roslaunch', 'multi_car_interaction', 'single_car_event_monitor.launch',
            f'race_mode:={race_mode}'
        ]

        print(f"📊 Launching race event monitor ({race_mode})...")

        # Launch in background (monitor should run silently)
        # Redirect to /dev/null to prevent blocking
        devnull = open(os.devnull, 'w')
        subprocess.Popen(
            cmd,
            stdout=devnull,
            stderr=devnull,
            preexec_fn=os.setsid
        )

        time.sleep(2)  # Give monitor time to initialize
        return True

    def set_simulation_id(self, simulation_id):
        """Set the /simulation_id and /batch_number ROS parameters"""
        try:
            rospy.set_param('race_test/simulation_id', simulation_id)
            # Ensure batch_number is always set (re-set in case it wasn't set earlier)
            rospy.set_param('race_test/batch_number', self.batch_number)
            print(
                f"✅ Set race_test/simulation_id to {simulation_id}, race_test/batch_number to {self.batch_number}")
            return True
        except Exception as e:
            print(f"⚠️  Error setting simulation_id/batch_number: {e}")
            return False

    def set_default_config_params(self, test_config):
        """Set default configuration parameters from YAML file to /race_test/ namespace"""
        if self.default_config is None:
            print("⚠️  No default configuration to set")
            return False

        print("⚙️  Setting default configuration parameters...")

        try:
            params_set = 0
            for param_name, param_value in self.default_config.items():
                param_path = f'/race_test/{param_name}'
                rospy.set_param(param_path, param_value)
                print(f"   ✅ {param_path} = {param_value}")
                params_set += 1

            print(f"   ✅ Set {params_set} default configuration parameters")
            return True

        except Exception as e:
            print(f"   ⚠️  Error setting default config parameters: {e}")
            import traceback
            traceback.print_exc()
            return False

    def set_global_prediction_switch(self, test_config):
        """Ensure /race_test/use_global_prediction is set for each test"""

        desired_value = test_config.get(
            'use_global_prediction', self.default_use_global_prediction)

        # Also set max speed limit for predictive spliner's global prediction
        # Use same map-specific logic as obstacle
        default_speed_limit = 10.0
        if test_config.get('global_map') == 'f':
            default_speed_limit = 6.35

        prediction_max_speed = test_config.get(
            'global_prediction_max_speed', default_speed_limit)

        try:
            rospy.set_param('/race_test/use_global_prediction', desired_value)
            rospy.set_param(
                '/race_test/global_prediction_max_speed', prediction_max_speed)
            print(
                f"   ✅ /race_test/use_global_prediction = {desired_value}")
            print(
                f"   ✅ /race_test/global_prediction_max_speed = {prediction_max_speed} m/s")
            return True
        except Exception as e:
            print(
                f"   ⚠️  Failed to set global prediction params: {e}")
            return False

    def set_planner_params_for_car(self, car, test_config, planner_name):
        """Set planner-specific parameters for a given car

        Args:
            car: Car identifier (e.g., 'car1', 'car2') or None for single car mode
            test_config: Test configuration dictionary
            planner_name: Name of the planner
        """
        print(
            f"⚙️  Setting planner parameters for {car if car else 'single car'} ({planner_name})...")

        # Build the parameter prefix to search for
        if car:
            param_prefix = f"{car}_{planner_name}_"
        else:
            param_prefix = f"{planner_name}_"

        # Find all parameters in test_config that match the pattern
        params = [key for key in test_config.keys(
        ) if key.startswith(param_prefix)]

        if not params:
            print(
                f"   ℹ️  No planner-specific parameters found for prefix '{param_prefix}'")
            return True

        # Set each parameter in ROS parameter server
        params_set = 0
        for param_key in params:
            # Extract the actual parameter name (remove the prefix)
            param_name = param_key[len(param_prefix):]
            param_value = test_config[param_key]

            # Build the ROS parameter path
            if car:
                param_path = f"/{car}/{param_name}"
            else:
                param_path = f"/{param_name}"

            try:
                rospy.set_param(param_path, param_value)
                print(f"   ✅ {param_path} = {param_value}")
                params_set += 1
            except Exception as e:
                print(f"   ⚠️  Failed to set {param_path}: {e}")

        print(f"   ✅ Set {params_set} planner-specific parameters")
        return True

    def set_planner_params(self, test_config):
        """Set planner parameters based on test mode"""
        if self.mode == "multi_car":
            # Multi-car mode: set parameters for both cars
            for car in ["car1", "car2"]:
                planner_name = test_config.get(f"planner_{car}", "")
                if planner_name:
                    self.set_planner_params_for_car(
                        car, test_config, planner_name)
        else:
            # Single-car mode: set parameters without car namespace
            planner_name = test_config.get("planner", "")
            if planner_name:
                self.set_planner_params_for_car(
                    None, test_config, planner_name)

    def check_overtaking_sectors(self):
        """Check that overtaking sectors are enabled (ot_flag set to true) for global and all cars"""
        print("🔍 Checking overtaking sectors...")

        try:
            # Define parameter namespaces to check: global + each car
            param_namespaces = ['/ot_map_params',
                                '/car1/ot_map_params', '/car2/ot_map_params']

            overall_success = True

            for namespace in param_namespaces:
                # Check if this namespace exists
                if not rospy.has_param(namespace):
                    print(f"   ⚠️  {namespace} not found - skipping")
                    continue

                # Get overtaking sector parameters for this namespace
                ot_params = rospy.get_param(namespace)
                n_sectors = ot_params.get('n_sectors', 0)

                if n_sectors == 0:
                    print(f"   ⚠️  {namespace}: No sectors defined")
                    continue

                # Display namespace being processed
                namespace_label = "Global" if namespace == '/ot_map_params' else namespace.split('/')[
                    1]
                print(
                    f"   [{namespace_label}] Found {n_sectors} overtaking sectors")

                # Check and enable each sector's ot_flag
                for i in range(n_sectors):
                    sector_key = f'Overtaking_sector{i}'
                    if sector_key in ot_params:
                        sector_params = ot_params[sector_key]
                        ot_flag = sector_params.get('ot_flag', False)

                        if ot_flag:
                            print(f"      ✅ Sector {i}: already enabled")
                        else:
                            print(
                                f"      🔧 Sector {i}: DISABLED - enabling now")
                            # Update the parameter
                            rospy.set_param(
                                f'{namespace}/{sector_key}/ot_flag', True)
                    else:
                        print(f"      ⚠️  Sector {i}: not found")
                        overall_success = False

            print("   ✅ All overtaking sectors checked and enabled")
            return overall_success

        except Exception as e:
            print(f"   ⚠️  Error checking overtaking sectors: {e}")
            import traceback
            traceback.print_exc()
            return False

    def wait_for_cars_ready(self, timeout=30):
        """Wait for both cars to be in READY state AND controller ready"""
        print("⏳ Waiting for cars to be READY...")

        start_time = time.time()
        cars = ['car1', 'car2']

        # Track which cars have set controller_ready parameter
        controller_ready = {car: False for car in cars}

        while True:
            elapsed = time.time() - start_time

            if elapsed > timeout:
                print(
                    f"   ⏱️  Timeout waiting for cars to be ready after {timeout}s")
                return False

            # Check state for both cars
            car_states = {}
            all_state_ready = True

            for car in cars:
                try:
                    # Try to get state from parameter or topic
                    state_param = f'/{car}/state_machine/current_state'

                    if rospy.has_param(state_param):
                        state = rospy.get_param(state_param, "UNKNOWN")
                    else:
                        # Assume ready if parameter doesn't exist after some time
                        if elapsed > 10:
                            state = "READY"
                        else:
                            state = "INITIALIZING"

                    car_states[car] = state

                    if state != "READY":
                        all_state_ready = False

                except Exception as e:
                    car_states[car] = "ERROR"
                    all_state_ready = False

            # Check for controller_ready parameter for each car
            for car in cars:
                if not controller_ready[car]:
                    controller_param = f'/{car}/controller_manager/controller_ready'
                    try:
                        if rospy.has_param(controller_param):
                            is_ready = rospy.get_param(controller_param, False)
                            if is_ready:
                                controller_ready[car] = True
                    except Exception as e:
                        # Parameter not yet set
                        pass

            # Print status
            if int(elapsed) % 5 == 0 and elapsed > 0:
                status_parts = []
                for car in cars:
                    state = car_states.get(car, "UNKNOWN")
                    ctrl_status = "✓" if controller_ready[car] else "✗"
                    status_parts.append(f"{car}: {state}|ctrl:{ctrl_status}")
                status_str = ", ".join(status_parts)
                print(f"   ... {status_str} ({elapsed:.0f}s / {timeout}s)")

            # Both conditions must be met: state READY AND controller ready
            if all_state_ready and all(controller_ready.values()):
                print(f"   ✅ Both cars are READY and controllers initialized!")
                return True

            time.sleep(0.5)

    def wait_for_single_car_ready(self, test_config, timeout=30):
        """Wait for single car (and optionally obstacle) to be READY"""
        print("⏳ Waiting for single car to be READY...")

        enable_obstacle = (test_config.get('mode') == 'single_car_obstacle')

        start_time = time.time()
        car_state_ready = False
        car_controller_ready = False
        obstacle_state_ready = False if enable_obstacle else True

        while True:
            elapsed = time.time() - start_time

            if elapsed > timeout:
                print(f"   ⏱️  Timeout waiting for readiness after {timeout}s")
                return False

            # Check car state (no namespace in single-car mode)
            try:
                state_param = '/state_machine/current_state'
                if rospy.has_param(state_param):
                    state = rospy.get_param(state_param, "UNKNOWN")
                    car_state_ready = (state == "READY")
                else:
                    # Assume ready if parameter doesn't exist after some time
                    if elapsed > 10:
                        car_state_ready = True
            except Exception as e:
                car_state_ready = False

            # Check car controller ready
            try:
                controller_param = '/controller_manager/controller_ready'
                if rospy.has_param(controller_param):
                    car_controller_ready = rospy.get_param(
                        controller_param, False)
            except Exception as e:
                pass

            # Check obstacle state if enabled (same logic as car state)
            if enable_obstacle:
                try:
                    obstacle_state_param = '/obstacle/state_machine/current_state'
                    if rospy.has_param(obstacle_state_param):
                        obstacle_state = rospy.get_param(
                            obstacle_state_param, "UNKNOWN")
                        obstacle_state_ready = (obstacle_state == "READY")
                    else:
                        # Assume ready if parameter doesn't exist after some time
                        if elapsed > 10:
                            obstacle_state_ready = True
                except Exception as e:
                    obstacle_state_ready = False

            # Print status periodically
            if int(elapsed) % 5 == 0 and elapsed > 0:
                status_parts = []
                status_parts.append(
                    f"car_state:{'✓' if car_state_ready else '✗'}")
                status_parts.append(
                    f"controller:{'✓' if car_controller_ready else '✗'}")
                if enable_obstacle:
                    status_parts.append(
                        f"obstacle:{'✓' if obstacle_state_ready else '✗'}")
                status_str = ", ".join(status_parts)
                print(f"   ... {status_str} ({elapsed:.0f}s / {timeout}s)")

            # Check if all required conditions are met
            if car_state_ready and car_controller_ready and obstacle_state_ready:
                print(f"   ✅ Single car ready (and obstacle if enabled)!")
                return True

            time.sleep(0.5)

    def start_race_via_service(self):
        """Start the race by calling /race_control/start_both service"""
        print("🏁 Starting race via service call...")

        try:
            # Wait for service to be available
            service_name = '/race_control/start_both'
            rospy.wait_for_service(service_name, timeout=10)

            # Call the service
            start_service = rospy.ServiceProxy(service_name, Empty)
            start_service()

            print("   ✅ Race started successfully!")
            return True

        except rospy.ServiceException as e:
            print(f"   ❌ Service call failed: {e}")
            return False
        except rospy.ROSException as e:
            print(f"   ⚠️  Service not available: {e}")
            return False

    def start_race_staggered(self, test_mode):
        """Start the race with staggered start: car2/obstacle first, then car1 after 3 seconds"""
        print("🏁 Starting race with staggered start...")

        try:
            if test_mode == 'multi_car':
                # Multi-car mode: start car2 first, then car1
                print("   🚗 Starting Car 2 first...")
                service_name = '/race_control/start_car2'
                rospy.wait_for_service(service_name, timeout=10)
                start_car2 = rospy.ServiceProxy(service_name, Empty)
                start_car2()
                print("   ✅ Car 2 started!")

                print("   ⏳ Waiting 3 seconds before starting Car 1...")
                time.sleep(3)

                print("   🚗 Starting Car 1...")
                service_name = '/race_control/start_car1'
                rospy.wait_for_service(service_name, timeout=10)
                start_car1 = rospy.ServiceProxy(service_name, Empty)
                start_car1()
                print("   ✅ Car 1 started!")

            elif test_mode == 'single_car_obstacle':
                # Single-car with obstacle mode: race controller handles obstacle via start_car2
                print("   🚧 Starting Obstacle first...")
                service_name = '/race_control/start_car2'
                rospy.wait_for_service(service_name, timeout=10)
                start_car2 = rospy.ServiceProxy(service_name, Empty)
                start_car2()
                print("   ✅ Obstacle started!")

                print("   ⏳ Waiting 3 seconds before starting Car...")
                time.sleep(3)

                print("   🚗 Starting Car...")
                service_name = '/race_control/start_car1'
                rospy.wait_for_service(service_name, timeout=10)
                start_car1 = rospy.ServiceProxy(service_name, Empty)
                start_car1()
                print("   ✅ Car started!")

            else:
                # For single_car_no_obstacle, use normal start
                return self.start_race_via_service()

            print("   ✅ Staggered race start complete!")
            return True

        except rospy.ServiceException as e:
            print(f"   ❌ Service call failed: {e}")
            return False
        except rospy.ROSException as e:
            print(f"   ⚠️  Service not available: {e}")
            return False

    def wait_for_completion(self, timeout=800):
        """
        Wait until /simulation_complete parameter is True or timeout

        Args:
            timeout: Maximum time to wait in seconds (default: 300s = 5 minutes)

        Returns:
            dict with status and elapsed time
        """

        # Initialize ROS node if not already initialized
        try:
            rospy.init_node('race_test_framework',
                            anonymous=True, disable_signals=True)
        except rospy.exceptions.ROSException:
            pass  # Node already initialized

        # Reset completion flag
        rospy.set_param('/race_test/simulation_complete', False)

        print("⏱️  Monitoring simulation...")
        print(
            f"   Waiting for race_test/simulation_complete parameter (timeout: {timeout}s)")

        start_time = time.time()

        while True:
            elapsed = time.time() - start_time

            # Check if simulation is complete
            try:
                is_complete = rospy.get_param(
                    'race_test/simulation_complete', False)

                if is_complete:
                    # Get completion reason if available
                    reason = rospy.get_param(
                        'race_test/race_complete_reason', 'unknown')
                    print(f"✅ Simulation complete after {elapsed:.1f}s")
                    print(f"   Reason: {reason}")
                    return {'status': 'complete', 'time': elapsed, 'reason': reason}

            except Exception as e:
                print(f"⚠️  Error reading parameter: {e}")

            # Check timeout
            if elapsed > timeout:
                print(f"⏱️  Timeout reached after {timeout}s")
                return {'status': 'timeout', 'time': elapsed, 'reason': 'timeout'}

            # Status update every 10 seconds
            if int(elapsed) % 10 == 0 and elapsed > 0:
                print(f"   ... still running ({elapsed:.0f}s / {timeout}s)")

            time.sleep(1)

    def terminate_simulation(self):
        """Terminate the roslaunch process and all child processes"""

        if self.launch_process is None:
            return

        print("🛑 Terminating simulation...")

        try:
            process_pgid = os.getpgid(self.launch_process.pid)

            # First attempt: Send SIGTERM for graceful shutdown
            print("   ⏳ Attempting graceful shutdown (SIGTERM)...")
            try:
                os.killpg(process_pgid, signal.SIGTERM)
            except ProcessLookupError:
                print("   ℹ️  Process already terminated")
                self.launch_process = None
                return

            # Wait for graceful shutdown (up to 30s like manual Ctrl+C)
            try:
                self.launch_process.wait(timeout=35)
                print("   ✅ Simulation terminated gracefully")
                self.launch_process = None
                return
            except subprocess.TimeoutExpired:
                print("   ⏱️  Graceful shutdown timed out after 30s, forcing kill...")

            # Second attempt: Force kill with SIGKILL
            try:
                os.killpg(process_pgid, signal.SIGKILL)
                self.launch_process.wait(timeout=5)
                print("   ✅ Simulation force-killed")
            except subprocess.TimeoutExpired:
                print("   ⚠️  Process group still not responding, using pkill...")
            except ProcessLookupError:
                print("   ✅ Process already terminated")

        except Exception as e:
            print(f"   ⚠️  Error during termination: {e}")

        finally:
            self.launch_process = None

        # Comprehensive cleanup: kill everything related to the simulation
        print("   🧹 Comprehensive cleanup...")

        # Kill Gazebo
        subprocess.run(['pkill', '-9', 'gzserver'],
                       stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL)
        subprocess.run(['pkill', '-9', 'gzclient'],
                       stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL)

        # Kill RViz
        subprocess.run(['pkill', '-9', 'rviz'],
                       stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL)

        # Kill all roslaunch processes
        subprocess.run(['pkill', '-9', 'roslaunch'],
                       stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL)

        # Kill all Python ROS nodes by name
        python_nodes = [
            'controller_manager', 'tracking.py', 'state_machine_node',
            'predictive_spliner', 'tam_sampling', 'race_event_monitor',
            'race_start_controller', 'frenet_converter', 'perception'
        ]
        for proc in python_nodes:
            subprocess.run(['pkill', '-9', '-f', proc],
                           stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL)

        # Kill any car-namespaced nodes
        subprocess.run(['pkill', '-9', '-f', '/car[12]/'],
                       stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL)

        print("   ✅ Comprehensive cleanup complete")
        time.sleep(3)  # Wait longer for processes to fully terminate

    def run_single_test(self, test_config, test_number, total_tests):
        """Run a single test configuration"""

        test_name = test_config['name']
        simulation_id = test_config.get('simulation_id', test_number)
        test_mode = test_config.get('mode', 'multi_car')

        print("\n" + "="*70)
        print(
            f"🏁 Test {test_number}/{total_tests}: {test_name} (ID: {simulation_id})")
        print("="*70)
        print(f"   Mode: {test_mode}")

        # Print mode-specific information
        if test_mode == 'multi_car':
            print(f"   Car 1: {test_config['planner_car1']} "
                  f"(speed: {test_config['speed_multiplier_car1']}x, "
                  f"accel: {test_config['accel_multiplier_car1']}x)")
            print(f"   Car 2: {test_config['planner_car2']} "
                  f"(speed: {test_config['speed_multiplier_car2']}x, "
                  f"accel: {test_config['accel_multiplier_car2']}x)")
        else:
            # Single-car mode
            print(f"   Planner: {test_config['planner']} "
                  f"(speed: {test_config.get('speed_multiplier', 1.0)}x, "
                  f"accel: {test_config.get('accel_multiplier', 1.0)}x)")
            if test_mode == 'single_car_obstacle':
                obs_traj = test_config.get('obstacle_trajectory', 'min_curv')
                obs_speed = test_config.get('obstacle_speed', 0.5)
                obs_start = test_config.get('obstacle_start_s', 0)
                obs_amp = test_config.get('obstacle_path_amplitude', 0.0)
                print(f"   Obstacle: trajectory={obs_traj}, speed={obs_speed}x, "
                      f"start_s={obs_start}m, amplitude={obs_amp}m")

        print(f"   Map: {test_config['global_map']}")
        print()

        # First, terminate any manually running simulations
        print("🧹 Checking for existing simulations...")
        self.terminate_existing_simulations()

        # Restart roscore for clean environment
        self.restart_roscore()

        # Set simulation ID parameter (before launching anything)
        self.set_simulation_id(simulation_id)

        # Set default configuration parameters (must be BEFORE launching monitor)
        self.set_default_config_params(test_config)

        # Enable predictive spliner switch so overtaking prediction is available immediately
        self.set_global_prediction_switch(test_config)

        # Set planner specific variable params
        # Set for car1 (and car2 if multi mode)
        self.set_planner_params(test_config)

        # Launch simulation based on mode
        if test_mode in ['single_car_no_obstacle', 'single_car_obstacle']:
            # Single-car mode
            if not self.launch_single_car_simulation(test_config):
                print(
                    f"❌ Failed to launch single-car simulation for {test_name}")
                return None

            # Launch race event monitor for single-car
            self.launch_race_event_monitor(test_config)
        else:
            # Multi-car mode
            if not self.launch_simulation(test_config):
                print(f"❌ Failed to launch simulation for {test_name}")
                return None

        # Mode-specific setup
        if test_mode == 'multi_car':
            # Check overtaking sectors are enabled (multi-car only)
            self.check_overtaking_sectors()

            # Wait for cars to be ready
            if not self.wait_for_cars_ready(timeout=30):
                print("⚠️  Cars did not reach READY state, starting race anyway...")
        else:
            # Single-car mode - wait for single car (and obstacle if enabled)
            if not self.wait_for_single_car_ready(test_config, timeout=30):
                print("⚠️  Car did not reach READY state, starting race anyway...")

        # Start the race with staggered start for multi-car and single-car-obstacle modes
        if test_mode in ['multi_car', 'single_car_obstacle']:
            if not self.start_race_staggered(test_mode):
                print("❌ Failed to start race with staggered start")
                # Terminate simulation and return None to skip this test
                self.terminate_simulation()
                print("❄️  Cooldown (10 seconds)...")
                time.sleep(10)
                return None
        else:
            # Single-car no obstacle: use simultaneous start
            if not self.start_race_via_service():
                print("❌ Failed to start race via service")
                # Terminate simulation and return None to skip this test
                self.terminate_simulation()
                print("❄️  Cooldown (10 seconds)...")
                time.sleep(10)
                return None

        # Wait for completion
        result = self.wait_for_completion(timeout=800)

        # Add test info to result
        result['test_name'] = test_name
        result['simulation_id'] = simulation_id
        result['config'] = test_config
        result['mode'] = test_mode

        # Terminate simulation
        self.terminate_simulation()

        # Cooldown between tests
        print("❄️  Cooling down (3 seconds)...")
        time.sleep(3)

        return result

    def run_all_tests(self):
        """Execute all tests in the configuration sequentially"""

        total_tests = len(self.test_configs)

        print(f"\n🎯 Starting {total_tests} sequential tests...\n")

        # Save config files to batch directory before starting tests
        self.save_config_files_to_batch_dir()

        for idx, test_config in enumerate(self.test_configs, start=1):
            result = self.run_single_test(test_config, idx, total_tests)

            if result:
                self.test_results.append(result)

        # Print summary
        self.print_summary()

    def print_summary(self):
        """Print test results summary"""

        print("\n" + "="*70)
        print("📊 TEST SUMMARY")
        print("="*70)
        print(f"Batch Number: {self.batch_number}")
        print()

        if not self.test_results:
            print("❌ No test results available")
            return

        for idx, result in enumerate(self.test_results, start=1):
            status_icon = "✅" if result['status'] == 'complete' else "⏱️"
            sim_id = result.get('simulation_id', 'N/A')
            reason = result.get('reason', 'N/A')
            mode = result.get('mode', 'multi_car')

            print(f"\n{idx}. {result['test_name']} (Simulation ID: {sim_id})")
            print(f"   Mode: {mode}")
            print(f"   Status: {status_icon} {result['status']}")
            print(f"   Reason: {reason}")
            print(f"   Time: {result['time']:.1f}s")

            # Print mode-specific configuration
            if mode == 'multi_car':
                print(f"   Car1: {result['config']['planner_car1']} vs "
                      f"Car2: {result['config']['planner_car2']}")
            else:
                print(f"   Planner: {result['config']['planner']}")
                if mode == 'single_car_obstacle':
                    print(
                        f"   Obstacle: {result['config'].get('obstacle_speed', 0.5)}x speed")

        print("\n" + "="*70)
        print(f"✅ Completed {len(self.test_results)} tests")
        print("="*70 + "\n")

    def cleanup(self):
        """Cleanup on exit"""
        print("\n🧹 Cleaning up...")
        self.terminate_simulation()


def parse_arguments():
    """Parse command-line arguments"""
    parser = argparse.ArgumentParser(
        description='Racing Simulation Test Framework - Run automated race tests',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run single-car time trial tests only:
  python3 race_test_framework.py --mode single_car_no_obstacle
  
  # Run single-car with obstacle tests only:
  python3 race_test_framework.py --mode single_car_obstacle
  
  # Run multi-car tests only:
  python3 race_test_framework.py --mode multi_car
  
  # Run all tests sequentially (time trial → obstacle → multi-car):
  python3 race_test_framework.py --mode all
  
  # Use custom config file:
  python3 race_test_framework.py --mode single_car_no_obstacle --config my_config.yaml
  
  # Run without RViz visualization:
  python3 race_test_framework.py --mode multi_car --rviz false
        """
    )

    parser.add_argument(
        '--mode',
        type=str,
        choices=['single_car_no_obstacle',
                 'single_car_obstacle', 'multi_car', 'all'],
        default='multi_car',
        help='Test mode to run (default: multi_car)'
    )

    parser.add_argument(
        '--config',
        type=str,
        default=None,
        help='Custom config file path (overrides mode-based config selection)'
    )

    parser.add_argument(
        '--rviz',
        type=str,
        choices=['true', 'false'],
        default='true',
        help='Enable RViz visualization (default: true)'
    )

    return parser.parse_args()


def main():
    """Main entry point"""

    args = parse_arguments()

    # Determine config file based on mode
    if args.config:
        config_file = args.config
    else:
        # Map mode to config file
        mode_to_config = {
            'single_car_no_obstacle': 'single_car_no_obstacle_config.yaml',
            'single_car_obstacle': 'single_car_with_obstacle_config.yaml',
            'multi_car': 'multi_car_config.yaml',
            'all': None  # Special handling for 'all' mode
        }
        config_file = mode_to_config.get(args.mode)

    # Create framework instance with mode and config
    if args.mode == 'all':
        framework = RaceTestFramework(mode='all', rviz=args.rviz)
    else:
        framework = RaceTestFramework(
            mode=args.mode, config_file=config_file, rviz=args.rviz)

    try:
        framework.run_all_tests()

    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user (Ctrl+C)")
        framework.cleanup()

    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        framework.cleanup()

    finally:
        print("\n👋 Test framework finished\n")


if __name__ == '__main__':
    main()
