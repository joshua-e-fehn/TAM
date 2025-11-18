#!/usr/bin/env python3
"""
Trajectory generation using TUM trajectory optimizer.
"""
import os
import sys
import yaml
import math
import shutil
import numpy as np
from typing import List, Dict
from config import Waypoint, MapConfig, OptimizationType, TRAJECTORY_OPTIMIZATION_PARAMS


class TrajectoryGenerator:
    """Generates optimized trajectories using TUM trajectory optimizer."""

    def __init__(self, config: MapConfig, cache_manager=None):
        self.config = config
        self.cache_manager = cache_manager

    def generate_shortest_path(self, centerline_waypoints: List[Waypoint],
                               trackbounds_left: List[Waypoint] = None,
                               trackbounds_right: List[Waypoint] = None) -> List[Waypoint]:
        """Generate shortest path using trajectory optimizer."""
        try:
            print("🔧 STARTING SHORTEST PATH GENERATION")
            print(
                f"   Using TUM trajectory optimizer with {len(centerline_waypoints)} centerline waypoints")
            print("   ⚙️  Setting up vehicle parameters and GGV diagram...")
            return self._run_trajectory_optimizer(centerline_waypoints, "shortest_path",
                                                  trackbounds_left, trackbounds_right)
        except Exception as e:
            print(f"❌ Shortest path generation FAILED: {e}")
            print("🔄 Applying fallback: moderate racing line as SP trajectory")
            return self._apply_velocity_optimization(centerline_waypoints, "moderate")

    def generate_racing_line(self, centerline_waypoints: List[Waypoint],
                             optimization_type: str = OptimizationType.MINTIME,
                             trackbounds_left: List[Waypoint] = None,
                             trackbounds_right: List[Waypoint] = None) -> List[Waypoint]:
        """Generate racing line using trajectory optimizer."""
        try:
            print(f"🏁 STARTING RACING LINE GENERATION")
            print(f"   Optimization type: {optimization_type.upper()}")
            print(f"   Car configuration: {self.config.car_name}")
            print(f"   Using {len(centerline_waypoints)} centerline waypoints")
            print("   ⚙️  Setting up vehicle parameters and optimization constraints...")
            racing_line = self._run_trajectory_optimizer(
                centerline_waypoints, optimization_type, trackbounds_left, trackbounds_right)

            # Apply velocity post-processing if needed
            if self._has_constant_velocity(racing_line):
                print(
                    "🔄 Detected constant velocity profile - applying curvature-based optimization")
                racing_line = self._apply_velocity_optimization(
                    racing_line, optimization_type)

            return racing_line

        except Exception as e:
            print(f"❌ Racing line generation FAILED: {e}")
            print("🔄 Returning None to allow fallback to input data")
            return None

    def _run_trajectory_optimizer(self, centerline_waypoints: List[Waypoint],
                                  optimization_type: str,
                                  trackbounds_left: List[Waypoint] = None,
                                  trackbounds_right: List[Waypoint] = None) -> List[Waypoint]:
        """Run the TUM trajectory optimizer."""
        # Create temporary files in current working directory
        # (the trajectory optimizer has a bug where it looks for files in cwd, not input_path)
        track_name = "temp_track_bounds"
        track_file = f"{track_name}.csv"
        self._create_track_bounds_file(track_file, centerline_waypoints,
                                       trackbounds_left, trackbounds_right)

        # Create vehicle parameters
        self._create_vehicle_parameters()
        self._create_ggv_diagram()
        self._create_ax_max_curve()

        try:
            # Import and run optimizer
            sys.path.append(
                '/home/atlas/catkin_ws/src/race_stack/planner/gb_optimizer/src')
            from global_racetrajectory_optimization.trajectory_optimizer import trajectory_optimizer

            print("   🎯 Calling TUM trajectory optimizer...")
            # Create a temporary directory for input files
            current_dir = os.getcwd()

            # Create veh_dyn_info directory if it doesn't exist (TUM optimizer expects files there)
            veh_dyn_dir = "/home/atlas/catkin_ws/veh_dyn_info"
            os.makedirs(veh_dyn_dir, exist_ok=True)

            # Copy GGV and vehicle parameter files to expected locations
            import shutil
            shutil.copy("ggv.csv", os.path.join(veh_dyn_dir, "ggv.csv"))
            shutil.copy("ax_max_machines.csv", os.path.join(
                veh_dyn_dir, "ax_max_machines.csv"))
            shutil.copy("racecar_f110.ini", os.path.join(
                veh_dyn_dir, "racecar_f110.ini"))

            # Load trajectory parameters for safety width
            traj_params = self._load_trajectory_optimization_parameters()
            print(
                f"   📐 Safety width: {traj_params['safety_width']}m, Optimization: {optimization_type}")

            optimized_trajectory, bound_r, bound_l, lap_time = trajectory_optimizer(
                input_path=current_dir,
                track_name=track_name,
                curv_opt_type=optimization_type,
                safety_width=traj_params["safety_width"],
                plot=False
            )

            print(f"   ⏱️  Optimization completed! Lap time: {lap_time:.3f}s")
            # Convert result to waypoints
            result = self._convert_optimizer_result(optimized_trajectory)
            print(f"   ✅ Converted to {len(result)} optimized waypoints")
            return result

        finally:
            # Cleanup temporary files
            print("   🧹 Cleaning up temporary files...")
            self._cleanup_temp_files(
                [track_file, "racecar_f110.ini", "ggv.csv", "ax_max_machines.csv"])

            # Also cleanup veh_dyn_info directory
            veh_dyn_dir = "/home/atlas/catkin_ws/veh_dyn_info"
            if os.path.exists(veh_dyn_dir):
                try:
                    shutil.rmtree(veh_dyn_dir)
                except:
                    pass  # Ignore cleanup errors

    def _create_track_bounds_file(self, filepath: str, centerline_waypoints: List[Waypoint],
                                  trackbounds_left: List[Waypoint] = None,
                                  trackbounds_right: List[Waypoint] = None):
        """Create track bounds file for optimizer using provided trackbounds data with improved accuracy."""
        with open(filepath, 'w') as f:
            f.write("# x_ref_m,y_ref_m,w_tr_right_m,w_tr_left_m\n")

            # If trackbounds are provided, use them to calculate track widths
            if trackbounds_left and trackbounds_right and len(trackbounds_left) > 0 and len(trackbounds_right) > 0:
                print(
                    f"   📏 Using provided trackbounds: {len(trackbounds_left)} left, {len(trackbounds_right)} right points")

                # Improved track width calculation with safety margins
                traj_params = self._load_trajectory_optimization_parameters()
                safety_margin = traj_params["safety_width"]

                for i, center_wp in enumerate(centerline_waypoints):
                    # Find closest left and right trackbound points using more robust method
                    min_left_dist = float('inf')
                    min_right_dist = float('inf')

                    # Check multiple nearby trackbound points for better accuracy
                    search_window = max(
                        1, min(10, len(trackbounds_left) // 20))

                    for left_wp in trackbounds_left:
                        dist = ((center_wp.x_m - left_wp.x_m)**2 +
                                (center_wp.y_m - left_wp.y_m)**2)**0.5
                        min_left_dist = min(min_left_dist, dist)

                    for right_wp in trackbounds_right:
                        dist = ((center_wp.x_m - right_wp.x_m)**2 +
                                (center_wp.y_m - right_wp.y_m)**2)**0.5
                        min_right_dist = min(min_right_dist, dist)

                    # Apply enhanced safety margins to prevent boundary violations
                    # Use more conservative safety margins to keep trajectories well within bounds
                    w_tr_left = max(
                        0.3, min_left_dist - safety_margin) if min_left_dist != float('inf') else 1.0
                    w_tr_right = max(
                        0.3, min_right_dist - safety_margin) if min_right_dist != float('inf') else 1.0

                    # Ensure minimum track width for feasible optimization
                    min_track_width = 0.5  # Minimum 0.5m on each side
                    w_tr_left = max(min_track_width, w_tr_left)
                    w_tr_right = max(min_track_width, w_tr_right)

                    f.write(
                        f"{center_wp.x_m:.6f},{center_wp.y_m:.6f},{w_tr_right:.6f},{w_tr_left:.6f}\n")

                print(
                    f"   ✅ Applied safety margin: {safety_margin}m, min track width: {min_track_width}m")
            else:
                # Fallback: Use d_left and d_right from centerline waypoints with safety margins
                print(
                    "   ⚠️  No trackbounds provided, using centerline waypoint distances with safety margins")
                traj_params = self._load_trajectory_optimization_parameters()
                safety_margin = traj_params["safety_width"]

                for wp in centerline_waypoints:
                    # Apply enhanced safety margins to existing distances
                    safe_d_left = max(0.3, wp.d_left - safety_margin * 1.5)
                    safe_d_right = max(0.3, wp.d_right - safety_margin * 1.5)

                    f.write(
                        f"{wp.x_m:.6f},{wp.y_m:.6f},{safe_d_right:.6f},{safe_d_left:.6f}\n")

            # Handle track closure based on rolling start setting
            if centerline_waypoints:
                first_wp = centerline_waypoints[0]

                # Check if rolling start is enabled
                rolling_start = traj_params.get("rolling_start", False)

                if not rolling_start:
                    # Traditional closed loop: Ensure proper track closure by repeating first point at end
                    if trackbounds_left and trackbounds_right and len(trackbounds_left) > 0 and len(trackbounds_right) > 0:
                        # Calculate distances for first waypoint (same logic as above)
                        min_left_dist = float('inf')
                        min_right_dist = float('inf')

                        for left_wp in trackbounds_left:
                            dist = ((first_wp.x_m - left_wp.x_m)**2 +
                                    (first_wp.y_m - left_wp.y_m)**2)**0.5
                            min_left_dist = min(min_left_dist, dist)

                        for right_wp in trackbounds_right:
                            dist = ((first_wp.x_m - right_wp.x_m)**2 +
                                    (first_wp.y_m - right_wp.y_m)**2)**0.5
                            min_right_dist = min(min_right_dist, dist)

                        # Use reduced safety margin consistently
                        safety_margin = traj_params["safety_width"]
                        w_tr_left = max(
                            0.3, min_left_dist - safety_margin) if min_left_dist != float('inf') else 1.0
                        w_tr_right = max(
                            0.3, min_right_dist - safety_margin) if min_right_dist != float('inf') else 1.0

                        # Ensure minimum track width
                        min_track_width = 0.5
                        w_tr_left = max(min_track_width, w_tr_left)
                        w_tr_right = max(min_track_width, w_tr_right)

                        f.write(
                            f"{first_wp.x_m:.6f},{first_wp.y_m:.6f},{w_tr_right:.6f},{w_tr_left:.6f}\n")
                    else:
                        # Use reduced safety margin consistently
                        safety_margin = traj_params["safety_width"]
                        safe_d_left = max(
                            0.3, first_wp.d_left - safety_margin)
                        safe_d_right = max(
                            0.3, first_wp.d_right - safety_margin)

                        f.write(
                            f"{first_wp.x_m:.6f},{first_wp.y_m:.6f},{safe_d_right:.6f},{safe_d_left:.6f}\n")

                    print(
                        "   🔗 Added track closure point for smooth closed-loop optimization")
                else:
                    # Rolling start: No closure point added, trajectory can have different start/end states
                    print(
                        "   🏁 Rolling start enabled: No track closure enforced - trajectory can have different start/end states")

    def _create_vehicle_parameters(self):
        """Create vehicle parameters file for optimizer."""
        car_params = self._load_car_parameters()
        traj_params = self._load_trajectory_optimization_parameters()

        # Create optimized parameters for trajectory optimization using car-specific values
        veh_params = {
            # Use car-specific max velocity
            "v_max": car_params.get("max_velocity", 50.0),
            "length": car_params.get("wheelbase", 0.307),
            "width": car_params.get("track_width_front", 0.281),
            "mass": car_params.get("mass", 3.54),
            "dragcoeff": traj_params["dragcoeff"],
            # Kinematic limit
            "curvlim": min(traj_params["curvlim"], 1.0 / (car_params.get("wheelbase", 0.307) * 0.5)),
            "g": traj_params["gravity"]
        }

        print(
            f"   DEBUG: TUM optimizer will use v_max = {veh_params['v_max']} m/s")

        vehicle_params_mintime = {
            "wheelbase_front": car_params.get("lf", car_params.get("wheelbase", 0.307) / 2),
            "wheelbase_rear": car_params.get("lr", car_params.get("wheelbase", 0.307) / 2),
            "track_width_front": car_params.get("track_width_front", 0.281),
            "track_width_rear": car_params.get("track_width_rear", 0.281),
            "cog_z": car_params.get("cg_height", 0.014),
            "I_z": car_params.get("moment_of_inertia", car_params.get("mass", 3.54) * 0.05),
            "liftcoeff_front": traj_params["liftcoeff_front"],
            "liftcoeff_rear": traj_params["liftcoeff_rear"],
            "k_brake_front": traj_params["k_brake_front"],
            "k_drive_front": traj_params["k_drive_front"],
            "k_roll": traj_params["k_roll"],
            "t_delta": car_params.get("steering_time_constant", traj_params["t_delta"]),
            "t_drive": traj_params["t_drive"],
            "t_brake": traj_params["t_brake"],
            "power_max": traj_params["power_max"],
            "f_drive_max": traj_params["f_drive_max"],
            "f_brake_max": traj_params["f_brake_max"],
            "delta_max": car_params.get("max_steering_angle", 0.4189)
        }

        tire_params_mintime = {
            "C_Sf": traj_params["C_Sf"],
            "C_Sr": traj_params["C_Sr"],
            "lam_muy_f": traj_params["lam_muy_f"],
            "lam_muy_r": traj_params["lam_muy_r"],
            "muy": traj_params["muy"],
            "camber": traj_params["camber"],
            "r_wheel": traj_params["wheel_radius"],
            "c_roll": traj_params["c_roll"],
            # Scale with actual mass
            "f_z0": traj_params["f_z0"] * (car_params.get("mass", 3.54) / 3.54),
            "B_front": traj_params["B_front"],
            "C_front": traj_params["C_front"],
            "eps_front": traj_params["eps_front"],
            "E_front": traj_params["E_front"],
            "B_rear": traj_params["B_rear"],
            "C_rear": traj_params["C_rear"],
            "eps_rear": traj_params["eps_rear"],
            "E_rear": traj_params["E_rear"]
        }

        # Write parameters to INI file
        with open("racecar_f110.ini", "w") as f:
            f.write("# F1Tenth vehicle parameters for trajectory optimization\n")
            f.write(f"# Generated for {self.config.car_name} configuration\n")
            f.write("# Generated automatically by trajectory_generator.py\n\n")

            f.write("[GENERAL_OPTIONS]\n")
            f.write('ggv_file="ggv.csv"\n')
            f.write('ax_max_machines_file="ax_max_machines.csv"\n\n')

            f.write(
                f'stepsize_opts={{"stepsize_prep": {traj_params["stepsize_prep"]}, "stepsize_reg": {traj_params["stepsize_reg"]}, "stepsize_interp_after_opt": {traj_params["stepsize_interp_after_opt"]}}}\n')
            f.write(
                f'reg_smooth_opts={{"k_reg": {traj_params["k_reg"]}, "s_reg": {traj_params["s_reg"]}}}\n')
            f.write(
                f'curv_calc_opts={{"d_preview_curv": {traj_params["d_preview_curv"]}, "d_review_curv": {traj_params["d_review_curv"]}, "d_preview_head": {traj_params["d_preview_head"]}, "d_review_head": {traj_params["d_review_head"]}}}\n\n')

            f.write("veh_params = {")
            for i, (key, value) in enumerate(veh_params.items()):
                if i > 0:
                    f.write(",\n              ")
                f.write(f'"{key}": {value}')
            f.write("}\n\n")

            f.write(
                'vel_calc_opts={"dyn_model_exp": 1.0, "vel_profile_conv_filt_window": null}\n\n')

            f.write("[OPTIMIZATION_OPTIONS]\n")
            f.write(
                f'optim_opts_shortest_path={{"width_opt": {traj_params["safety_width"]}}}\n')
            f.write(
                f'optim_opts_mincurv={{"width_opt": {traj_params["safety_width"]}, "iqp_iters_min": 3, "iqp_curverror_allowed": 0.01}}\n')
            f.write(f'optim_opts_mintime={{"width_opt": {traj_params["safety_width"]}, "penalty_delta": {traj_params["penalty_delta"]}, "penalty_F": {traj_params["penalty_F"]}, "mue": {traj_params["friction_coeff"]}, "n_gauss": 5, "dn": 0.25, "limit_energy": false, "energy_limit": 2.0, "safe_traj": false, "ax_pos_safe": null, "ax_neg_safe": null, "ay_safe": null, "w_tr_reopt": 2.0, "w_veh_reopt": {traj_params["safety_width"]}, "w_add_spl_regr": 0.2, "step_non_reg": 0, "eps_kappa": 1e-3}}\n\n')

            f.write("vehicle_params_mintime = {")
            for i, (key, value) in enumerate(vehicle_params_mintime.items()):
                if i > 0:
                    f.write(",\n                          ")
                f.write(f'"{key}": {value}')
            f.write("}\n\n")

            f.write("tire_params_mintime = {")
            for i, (key, value) in enumerate(tire_params_mintime.items()):
                if i > 0:
                    f.write(",\n                       ")
                f.write(f'"{key}": {value}')
            f.write("}\n\n")

            # Add power parameters for mintime optimization
            f.write(
                'pwr_params_mintime = {"pwr_behavior": false, "simple_loss": true}\n')

        print(
            f"   ✅ Created vehicle parameter file with {len(veh_params)} general parameters")
        print(
            f"      Mass: {veh_params['mass']}kg, Max velocity: {veh_params['v_max']}m/s")
        print(
            f"      Power: {vehicle_params_mintime['power_max']}W, Safety width: {traj_params['safety_width']}m")

    def _create_ggv_diagram(self):
        """Create GGV diagram for optimizer."""
        car_params = self._load_car_parameters()
        traj_params = self._load_trajectory_optimization_parameters()

        # Appropriate GGV diagram for F1Tenth using configurable parameters
        ggv_data = []
        # Use actual car max velocity
        max_velocity = car_params.get("max_velocity", 50.0)

        print(f"   DEBUG: Creating GGV diagram up to {max_velocity} m/s")

        # Generate GGV data for F1Tenth scale velocities and accelerations
        for v in np.linspace(0.5, max_velocity * 1.5, 30):
            # Acceleration limits decrease with speed due to aerodynamic effects and tire limits
            # Use car-specific acceleration limits
            max_accel = car_params.get(
                "max_accel", traj_params["max_longitudinal_accel"])
            max_lateral = traj_params.get(
                "max_lateral_accel", traj_params["max_lateral_accel"])

            # Longitudinal acceleration - decreases with speed
            ax_max = max(traj_params["min_accel"], max_accel *
                         (1.0 - v / traj_params["accel_speed_factor"]))
            # Lateral acceleration - decreases with speed
            ay_max = max(traj_params["min_accel"], max_lateral *
                         (1.0 - v / traj_params["lateral_speed_factor"]))
            ggv_data.append([v, ax_max, ay_max])

        with open("ggv.csv", "w") as f:
            f.write("# vx_mps,ax_max_mps2,ay_max_mps2\n")
            for row in ggv_data:
                f.write(f"{row[0]:.2f},{row[1]:.2f},{row[2]:.2f}\n")

        print(
            f"   ✅ Created GGV diagram with max accel: {max_accel:.1f}m/s², max lateral: {max_lateral:.1f}m/s²")

    def _create_ax_max_curve(self):
        """Create ax_max curve for optimizer using dynamic parameters."""
        # Load trajectory parameters for power curve configuration
        traj_params = self._load_trajectory_optimization_parameters()
        car_params = self._load_car_parameters()

        ax_data = []
        max_velocity = car_params.get("max_velocity", 50.0)

        print(f"   DEBUG: Creating ax_max curve up to {max_velocity} m/s")

        # Generate acceleration limits based on configurable power constraints
        for v in np.linspace(0.5, max_velocity * 1.2, 30):
            # Configurable power limitation: P/v with limits
            # Use max_longitudinal_accel as default to ensure consistency
            max_power_accel = traj_params.get("power_curve_max_accel",
                                              traj_params.get("max_longitudinal_accel", 3.0))
            power_factor = traj_params.get("power_curve_factor", 50.0)
            power_limited_acc = min(
                max_power_accel, power_factor / max(v, 1.0))

            # Configurable friction-limited acceleration
            friction_limited_acc = traj_params.get(
                "friction_limited_accel", traj_params.get("max_longitudinal_accel", 3.0))

            ax_max = min(power_limited_acc, friction_limited_acc)
            ax_data.append([v, ax_max])

        with open("ax_max_machines.csv", "w") as f:
            f.write("# vx_mps,ax_max_mps2\n")
            for row in ax_data:
                f.write(f"{row[0]:.2f},{row[1]:.2f}\n")

    def _load_car_parameters(self) -> dict:
        """Load comprehensive car parameters from all available configuration files."""
        car_config_path = os.path.expanduser(
            f"~/catkin_ws/src/race_stack/stack_master/config/{self.config.car_name}")

        car_params = {}

        # 1. Load basic car model parameters
        car_model_file = os.path.join(car_config_path, "car_model.yaml")
        if os.path.exists(car_model_file):
            try:
                with open(car_model_file, 'r') as f:
                    config_data = yaml.safe_load(f)

                # Basic vehicle parameters
                car_params["mass"] = config_data.get("m", 3.54)
                car_params["moment_of_inertia"] = config_data.get(
                    "Iz", 0.05797)
                car_params["lf"] = config_data.get("lf", 0.162)
                car_params["lr"] = config_data.get("lr", 0.145)
                car_params["wheelbase"] = config_data.get(
                    "wheelbase", car_params["lf"] + car_params["lr"])
                car_params["cg_height"] = config_data.get("h_cg", 0.014)
                car_params["max_velocity"] = config_data.get(
                    "v_max", 50.0)  # Default to reasonable racing speed
                car_params["min_velocity"] = config_data.get("v_min", -5.0)
                car_params["max_accel"] = config_data.get("a_max", 3.0)
                car_params["min_accel"] = config_data.get("a_min", -3.0)
                car_params["max_steering_angle"] = config_data.get(
                    "max_steering_angle", 0.4189)
                car_params["max_steering_velocity"] = config_data.get(
                    "max_steering_velocity", 3.2)
                car_params["steering_time_constant"] = config_data.get(
                    "tau_steer", 0.15779476)

                # Control parameters
                car_params["C_0d"] = config_data.get("C_0d", 0.48)
                car_params["C_d"] = config_data.get("C_d", -1.1)
                car_params["C_acc"] = config_data.get("C_acc", 8.29)
                car_params["C_dec"] = config_data.get("C_dec", 5.77)
                car_params["C_R"] = config_data.get("C_R", 2.03)
                car_params["C_0v"] = config_data.get("C_0v", 100)
                car_params["C_v"] = config_data.get("C_v", 20)

                print(f"✓ Loaded basic car parameters from {car_model_file}")
                print(
                    f"   DEBUG: Car {self.config.car_name} max_velocity = {car_params['max_velocity']} m/s")

            except Exception as e:
                print(f"Warning: Failed to load car model: {e}")

        # 2. Load Pacejka tire model parameters
        pacejka_file = os.path.join(
            car_config_path, f"{self.config.car_name}_pacejka.yaml")
        if os.path.exists(pacejka_file):
            try:
                with open(pacejka_file, 'r') as f:
                    pacejka_data = yaml.safe_load(f)

                # Pacejka tire coefficients
                car_params["tire_model"] = pacejka_data.get(
                    "tire_model", "linear")
                car_params["mu"] = pacejka_data.get("mu", 1.0)
                car_params["C_Pf"] = pacejka_data.get(
                    "C_Pf", [4.798, 2.164, 0.650, 0.373])  # Front tire coefficients
                car_params["C_Pr"] = pacejka_data.get(
                    "C_Pr", [20.0, 1.5, 0.618, 0.0])      # Rear tire coefficients

                # Override control parameters if more accurate in Pacejka file
                if "C_acc" in pacejka_data:
                    car_params["C_acc"] = pacejka_data["C_acc"]
                if "C_dec" in pacejka_data:
                    car_params["C_dec"] = pacejka_data["C_dec"]
                if "C_R" in pacejka_data:
                    car_params["C_R"] = pacejka_data["C_R"]

                print(f"✓ Loaded Pacejka tire parameters from {pacejka_file}")

            except Exception as e:
                print(f"Warning: Failed to load Pacejka parameters: {e}")

        # 3. Load VESC motor controller parameters
        vesc_file = os.path.join(car_config_path, "vesc.yaml")
        if os.path.exists(vesc_file):
            try:
                with open(vesc_file, 'r') as f:
                    vesc_data = yaml.safe_load(f)

                # Motor parameters
                car_params["speed_to_erpm_gain"] = vesc_data.get(
                    "speed_to_erpm_gain", 4352)
                car_params["speed_to_erpm_offset"] = vesc_data.get(
                    "speed_to_erpm_offset", 220.0)
                car_params["max_servo_speed"] = vesc_data.get(
                    "max_servo_speed", 3.2)
                car_params["max_acceleration"] = vesc_data.get(
                    "max_acceleration", 2.5)
                car_params["max_accel_current"] = vesc_data.get(
                    "max_accel_current", 65)
                car_params["max_brake_current"] = vesc_data.get(
                    "max_brake_current", 50)
                car_params["current_time_constant"] = vesc_data.get(
                    "current_time_constant", 0.05)

                # Steering calibration
                car_params["steering_angle_to_servo_gain"] = vesc_data.get(
                    "steering_angle_to_servo_gain", -1.1)
                car_params["steering_angle_to_servo_offset"] = vesc_data.get(
                    "steering_angle_to_servo_offset", 0.495)

                # Motor control gains
                car_params["acceleration_to_current_gain"] = vesc_data.get(
                    "acceleration_to_current_gain", 8.938)
                car_params["deceleration_to_current_gain"] = vesc_data.get(
                    "deceleration_to_current_gain", 5.936)
                car_params["velocity_to_current_gain"] = vesc_data.get(
                    "velocity_to_current_gain", 3.693)

                # Override wheelbase if specified
                if "wheelbase" in vesc_data:
                    car_params["wheelbase"] = vesc_data["wheelbase"]

                print(f"✓ Loaded VESC motor parameters from {vesc_file}")

            except Exception as e:
                print(f"Warning: Failed to load VESC parameters: {e}")

        # 4. Load MPC controller parameters
        mpc_files = ["kinematic_mpc_params.yaml",
                     "single_track_mpc_params.yaml"]
        for mpc_file in mpc_files:
            mpc_path = os.path.join(car_config_path, mpc_file)
            if os.path.exists(mpc_path):
                try:
                    with open(mpc_path, 'r') as f:
                        mpc_data = yaml.safe_load(f)

                    # MPC constraints that affect trajectory optimization
                    car_params[f"mpc_{mpc_file.split('_')[0]}_v_min"] = mpc_data.get(
                        "v_min", 2.0)
                    car_params[f"mpc_{mpc_file.split('_')[0]}_v_max"] = mpc_data.get(
                        "v_max", 12.0)
                    car_params[f"mpc_{mpc_file.split('_')[0]}_a_min"] = mpc_data.get(
                        "a_min", -10.0)
                    car_params[f"mpc_{mpc_file.split('_')[0]}_a_max"] = mpc_data.get(
                        "a_max", 10.0)
                    car_params[f"mpc_{mpc_file.split('_')[0]}_alat_max"] = mpc_data.get(
                        "alat_max", 10.0)
                    car_params[f"mpc_{mpc_file.split('_')[0]}_delta_min"] = mpc_data.get(
                        "delta_min", -0.40)
                    car_params[f"mpc_{mpc_file.split('_')[0]}_delta_max"] = mpc_data.get(
                        "delta_max", 0.40)

                    # MPC timing parameters
                    car_params[f"mpc_{mpc_file.split('_')[0]}_freq"] = mpc_data.get(
                        "MPC_freq", 20)
                    car_params[f"mpc_{mpc_file.split('_')[0]}_safety_margin"] = mpc_data.get(
                        "track_safety_margin", 0.3)

                    print(
                        f"✓ Loaded {mpc_file.replace('.yaml', '')} parameters")

                except Exception as e:
                    print(f"Warning: Failed to load {mpc_file}: {e}")

        # 5. Load L1 controller parameters
        l1_file = os.path.join(car_config_path, "l1_params.yaml")
        if os.path.exists(l1_file):
            try:
                with open(l1_file, 'r') as f:
                    l1_data = yaml.safe_load(f)

                # L1 controller parameters
                car_params["l1_t_clip_min"] = l1_data.get("t_clip_min", 0.8)
                car_params["l1_t_clip_max"] = l1_data.get("t_clip_max", 5.0)
                car_params["l1_m_l1"] = l1_data.get("m_l1", 0.6)
                car_params["l1_q_l1"] = l1_data.get("q_l1", -0.165)
                car_params["l1_speed_lookahead"] = l1_data.get(
                    "speed_lookahead", 0.25)
                car_params["l1_acc_scaler"] = l1_data.get(
                    "acc_scaler_for_steer", 1.2)
                car_params["l1_dec_scaler"] = l1_data.get(
                    "dec_scaler_for_steer", 0.9)

                print(f"✓ Loaded L1 controller parameters from {l1_file}")

            except Exception as e:
                print(f"Warning: Failed to load L1 parameters: {e}")

        # Set default track widths if not specified
        car_params.setdefault("track_width_front", 0.281)
        car_params.setdefault("track_width_rear", 0.281)

        print(
            f"✓ Comprehensive parameter loading complete for {self.config.car_name}")
        print(f"  Total parameters loaded: {len(car_params)}")
        print(
            f"  Mass: {car_params.get('mass', 'N/A')}kg, Wheelbase: {car_params.get('wheelbase', 'N/A')}m")
        print(
            f"  Max velocity: {car_params.get('max_velocity', 'N/A')}m/s, Max current: {car_params.get('max_accel_current', 'N/A')}A")

        return car_params

    def _load_trajectory_optimization_parameters(self) -> dict:
        """Load trajectory optimization parameters dynamically from comprehensive car configuration."""
        # Start with default parameters
        traj_params = TRAJECTORY_OPTIMIZATION_PARAMS["DEFAULT"].copy()

        # Load comprehensive car parameters from all config files
        car_params = self._load_car_parameters()

        # === DYNAMIC MAPPING FROM ALL CAR CONFIG FILES ===

        # 1. Basic vehicle dynamics from car_model.yaml
        if "max_accel" in car_params:
            base_accel = car_params["max_accel"]
            # Use MPC parameters if available for more accurate limits
            if "mpc_kinematic_alat_max" in car_params:
                traj_params["max_lateral_accel"] = car_params["mpc_kinematic_alat_max"]
            else:
                traj_params["max_lateral_accel"] = base_accel * 2.5

            if "mpc_kinematic_a_max" in car_params:
                traj_params["max_longitudinal_accel"] = car_params["mpc_kinematic_a_max"]
            else:
                traj_params["max_longitudinal_accel"] = base_accel * 2.5

            traj_params["friction_limited_accel"] = base_accel * 2.0

        # 2. Velocity limits from car model (don't constrain by MPC limits for trajectory optimization)
        if "max_velocity" in car_params:
            traj_params["v_max"] = car_params["max_velocity"]

        # Note: We do NOT apply MPC velocity constraints to trajectory optimization
        # MPC limits are for real-time control safety, but trajectory optimization
        # should explore the vehicle's full performance envelope

        # if "mpc_kinematic_v_max" in car_params:
        #     traj_params["v_max"] = min(
        #         traj_params["v_max"], car_params["mpc_kinematic_v_max"])
        if "mpc_kinematic_v_min" in car_params:
            traj_params["min_velocity"] = max(
                0.5, car_params["mpc_kinematic_v_min"])

        # 3. Steering and curvature limits
        if "max_steering_angle" in car_params and "wheelbase" in car_params:
            wheelbase = car_params["wheelbase"]
            max_steer = car_params["max_steering_angle"]
            # Physical curvature limit based on Ackermann geometry
            traj_params["curvlim"] = min(
                0.35, abs(math.tan(max_steer) / wheelbase))

        # 4. Power and force parameters from VESC configuration
        if "max_accel_current" in car_params and "acceleration_to_current_gain" in car_params:
            # Calculate power based on motor current capabilities
            max_current = car_params["max_accel_current"]  # Amperes
            current_gain = car_params["acceleration_to_current_gain"]

            # Estimate max force from current (simplified motor model)
            # Force ≈ (Current / Gain) * Mass * Efficiency
            mass = car_params.get("mass", 3.54)
            estimated_max_force = (max_current / current_gain) * mass * 0.85
            traj_params["f_drive_max"] = estimated_max_force

            # Brake force from brake current
            if "max_brake_current" in car_params and "deceleration_to_current_gain" in car_params:
                brake_current = car_params["max_brake_current"]
                brake_gain = car_params["deceleration_to_current_gain"]
                estimated_brake_force = (
                    brake_current / brake_gain) * mass * 0.85
                traj_params["f_brake_max"] = estimated_brake_force

        # 5. Power curve parameters from VESC motor characteristics
        if "speed_to_erpm_gain" in car_params and "max_accel_current" in car_params:
            # Motor power estimation from ERPM and current
            erpm_gain = car_params["speed_to_erpm_gain"]
            max_current = car_params["max_accel_current"]

            # Typical VESC voltage ~12-24V, power = V * I
            estimated_voltage = 18.0  # Conservative estimate
            traj_params["power_max"] = estimated_voltage * max_current

            # Power curve factor based on motor characteristics
            # Higher ERPM gain means motor winds up faster -> different power curve
            traj_params["power_curve_factor"] = max(30.0, erpm_gain / 100.0)

            # Max acceleration limited by current
            if "acceleration_to_current_gain" in car_params:
                current_to_accel = 1.0 / \
                    car_params["acceleration_to_current_gain"]
                traj_params["power_curve_max_accel"] = max_current * \
                    current_to_accel

        # 6. Tire model parameters from Pacejka configuration
        if "tire_model" in car_params and car_params["tire_model"] == "pacejka":
            if "mu" in car_params:
                traj_params["muy"] = car_params["mu"]
                traj_params["friction_coeff"] = car_params["mu"]

            # Pacejka coefficients for more accurate tire modeling
            if "C_Pf" in car_params:
                # Front tire magic formula coefficients [D, C, B, E]
                C_Pf = car_params["C_Pf"]
                if len(C_Pf) >= 4:
                    traj_params["C_Sf"] = C_Pf[0]  # Peak friction coefficient
                    traj_params["B_front"] = C_Pf[2]  # Stiffness factor
                    traj_params["C_front"] = C_Pf[1]  # Shape factor
                    traj_params["E_front"] = C_Pf[3]  # Curvature factor

            if "C_Pr" in car_params:
                # Rear tire magic formula coefficients
                C_Pr = car_params["C_Pr"]
                if len(C_Pr) >= 4:
                    traj_params["C_Sr"] = C_Pr[0]
                    traj_params["B_rear"] = C_Pr[2]
                    traj_params["C_rear"] = C_Pr[1]
                    traj_params["E_rear"] = C_Pr[3]

        # 7. Time constants from vehicle dynamics
        if "steering_time_constant" in car_params:
            traj_params["t_delta"] = car_params["steering_time_constant"]

        if "current_time_constant" in car_params:
            traj_params["t_drive"] = car_params["current_time_constant"]
            traj_params["t_brake"] = car_params["current_time_constant"]

        # 8. Safety margins from MPC configuration
        if "mpc_kinematic_safety_margin" in car_params:
            traj_params["safety_width"] = car_params["mpc_kinematic_safety_margin"]

        # 9. L1 controller influence on trajectory optimization
        if "l1_speed_lookahead" in car_params:
            # L1 lookahead affects how aggressive the trajectory can be
            lookahead = car_params["l1_speed_lookahead"]
            # More lookahead allows more aggressive optimization
            traj_params["lateral_accel_factor_moderate"] = min(
                1.0, 0.6 + lookahead * 1.6)

        if "l1_acc_scaler" in car_params and "l1_dec_scaler" in car_params:
            # L1 scaling factors influence optimization aggressiveness
            acc_scale = car_params["l1_acc_scaler"]
            dec_scale = car_params["l1_dec_scaler"]
            traj_params["accel_speed_factor"] = 25.0 / \
                acc_scale  # Inverse relationship
            traj_params["lateral_speed_factor"] = 30.0 / acc_scale

        # 10. Mass-based scaling for forces
        # Relative to NUC2 baseline
        mass_ratio = car_params.get("mass", 3.54) / 3.54
        traj_params["f_z0"] = traj_params["f_z0"] * \
            mass_ratio  # Normal force scales with mass

        # 11. Rolling start configuration from MapConfig
        if hasattr(self.config, 'rolling_start'):
            traj_params["rolling_start"] = self.config.rolling_start
            traj_params["initial_velocity"] = self.config.initial_velocity
            print(
                f"  Rolling start: {'ENABLED' if self.config.rolling_start else 'DISABLED'}")
            if self.config.rolling_start:
                print(
                    f"  Initial velocity: {self.config.initial_velocity:.1f}m/s")

        print(
            f"✓ Comprehensive trajectory optimization mapping complete for {self.config.car_name}")
        print(
            f"  Max lateral accel: {traj_params['max_lateral_accel']:.1f}m/s²")
        print(
            f"  Max power: {traj_params['power_max']:.0f}W, Max drive force: {traj_params['f_drive_max']:.0f}N")
        print(
            f"  Curvature limit: {traj_params['curvlim']:.3f}rad/m, Tire friction: {traj_params['muy']:.2f}")
        print(
            f"  Power curve: max_accel={traj_params.get('power_curve_max_accel', 'DEFAULT'):.1f}m/s², factor={traj_params.get('power_curve_factor', 'DEFAULT'):.0f}")

        return traj_params

    def _convert_optimizer_result(self, trajectory_data) -> List[Waypoint]:
        """Convert optimizer output to waypoint objects with improved closure continuity."""
        if trajectory_data is None or len(trajectory_data) == 0:
            print("Warning: Empty trajectory data received from optimizer")
            return []

        print(f"Converting optimizer result to waypoints...")
        print(f"Trajectory data shape: {trajectory_data.shape}")

        waypoints = []

        # The trajectory_optimizer returns data with columns:
        # [s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2]

        for i, row in enumerate(trajectory_data):
            if len(row) >= 7:  # Ensure we have all required columns
                waypoint = Waypoint(
                    id=i,
                    s_m=float(row[0]),      # s_m
                    # d_m (lateral displacement, set to 0 for optimized trajectory)
                    d_m=0.0,
                    x_m=float(row[1]),      # x_m
                    y_m=float(row[2]),      # y_m
                    d_right=1.0,            # Will be updated later if needed
                    d_left=1.0,             # Will be updated later if needed
                    psi_rad=float(row[3]),  # psi_rad
                    kappa_radpm=float(row[4]),  # kappa_radpm
                    vx_mps=float(row[5]),   # vx_mps
                    ax_mps2=float(row[6])   # ax_mps2
                )
                waypoints.append(waypoint)
            else:
                print(
                    f"Warning: Row {i} has insufficient data ({len(row)} columns)")

        # Improve track closure by ensuring smooth start/end connection
        if len(waypoints) > 3:
            waypoints = self._ensure_trajectory_closure(waypoints)

        # Validate trajectory stays within track boundaries
        waypoints = self._validate_trajectory_boundaries(waypoints)

        print(f"✓ Converted {len(waypoints)} waypoints from optimizer result")
        return waypoints

    def _ensure_trajectory_closure(self, waypoints: List[Waypoint]) -> List[Waypoint]:
        """Ensure trajectory has smooth closure by matching derivatives at start/end, or handle rolling start."""
        if len(waypoints) < 6:  # Need at least 6 points for proper interpolation
            return waypoints

        # Check if rolling start is enabled
        traj_params = self._load_trajectory_optimization_parameters()
        rolling_start = traj_params.get("rolling_start", False)

        if rolling_start:
            print(
                "   🏁 Rolling start mode: Applying initial velocity and no closure enforcement")

            # Set initial velocity for rolling start if specified
            initial_velocity = traj_params.get("initial_velocity", 5.0)
            # Only if current velocity is much lower
            if waypoints[0].vx_mps < initial_velocity * 0.5:
                print(
                    f"   🚀 Setting initial velocity to {initial_velocity:.1f} m/s for rolling start")
                waypoints[0] = Waypoint(
                    id=waypoints[0].id, s_m=waypoints[0].s_m, d_m=waypoints[0].d_m,
                    x_m=waypoints[0].x_m, y_m=waypoints[0].y_m,
                    d_right=waypoints[0].d_right, d_left=waypoints[0].d_left,
                    psi_rad=waypoints[0].psi_rad, kappa_radpm=waypoints[0].kappa_radpm,
                    vx_mps=initial_velocity, ax_mps2=waypoints[0].ax_mps2
                )

            # No closure enforcement for rolling start - start and end can be different
            return waypoints

        print("   🔗 Ensuring smooth trajectory closure with derivative matching...")

        # Get first and last few points for analysis
        first_point = waypoints[0]
        last_point = waypoints[-1]

        # Calculate position difference between start and end
        position_diff = math.sqrt((last_point.x_m - first_point.x_m)**2 +
                                  (last_point.y_m - first_point.y_m)**2)

        # Calculate derivative vectors at start and end
        # Use next few points to estimate derivatives
        # Use 3 points or 10% of trajectory
        n_points = min(3, len(waypoints) // 10)

        # Forward derivative at start (using points 0,1,2)
        start_dx = waypoints[2].x_m - waypoints[0].x_m
        start_dy = waypoints[2].y_m - waypoints[0].y_m
        start_heading_est = math.atan2(start_dy, start_dx)

        # Backward derivative at end (using points -3,-2,-1)
        end_dx = waypoints[-1].x_m - waypoints[-3].x_m
        end_dy = waypoints[-1].y_m - waypoints[-3].y_m
        end_heading_est = math.atan2(end_dy, end_dx)

        # Calculate heading difference
        heading_diff = abs(end_heading_est - start_heading_est)
        heading_diff = min(heading_diff, 2*math.pi -
                           heading_diff)  # Take smaller angle

        # Calculate curvature differences
        curvature_diff = abs(last_point.kappa_radpm - first_point.kappa_radpm)

        # Calculate velocity differences
        velocity_diff = abs(last_point.vx_mps - first_point.vx_mps)

        print(f"   📊 Closure analysis: pos={position_diff:.3f}m, heading={heading_diff:.3f}rad, " +
              f"curvature={curvature_diff:.3f}rad/m, velocity={velocity_diff:.3f}m/s")

        # Apply smooth blending for closure if position gap is reasonable
        if position_diff < 0.5:  # Only if trajectory is reasonably closed
            # Blend more points for smoother transition
            # Blend up to 5 points or 12.5% of trajectory
            blend_points = min(5, len(waypoints) // 8)

            for i in range(blend_points):
                point_idx = len(waypoints) - 1 - i
                blend_factor = (i + 1) / (blend_points + 1)  # Gradual blending

                current_wp = waypoints[point_idx]

                # Blend heading towards first point's heading (consider periodicity)
                target_heading = first_point.psi_rad
                current_heading = current_wp.psi_rad

                # Handle angle wrapping
                heading_diff_raw = target_heading - current_heading
                if heading_diff_raw > math.pi:
                    heading_diff_raw -= 2*math.pi
                elif heading_diff_raw < -math.pi:
                    heading_diff_raw += 2*math.pi

                blended_heading = current_heading + blend_factor * heading_diff_raw

                # Blend curvature towards first point's curvature
                target_curvature = first_point.kappa_radpm
                blended_curvature = current_wp.kappa_radpm + blend_factor * \
                    (target_curvature - current_wp.kappa_radpm)

                # Blend velocity towards first point's velocity
                target_velocity = first_point.vx_mps
                blended_velocity = current_wp.vx_mps + blend_factor * \
                    (target_velocity - current_wp.vx_mps)

                # Create updated waypoint
                waypoints[point_idx] = Waypoint(
                    id=current_wp.id, s_m=current_wp.s_m, d_m=current_wp.d_m,
                    x_m=current_wp.x_m, y_m=current_wp.y_m,
                    d_right=current_wp.d_right, d_left=current_wp.d_left,
                    psi_rad=blended_heading, kappa_radpm=blended_curvature,
                    vx_mps=blended_velocity, ax_mps2=current_wp.ax_mps2
                )

            # Verify closure after blending
            final_heading_diff = abs(
                waypoints[-1].psi_rad - waypoints[0].psi_rad)
            final_heading_diff = min(
                final_heading_diff, 2*math.pi - final_heading_diff)
            final_curvature_diff = abs(
                waypoints[-1].kappa_radpm - waypoints[0].kappa_radpm)
            final_velocity_diff = abs(
                waypoints[-1].vx_mps - waypoints[0].vx_mps)

            print(
                f"   ✅ Applied smooth closure blending to last {blend_points} points")
            print(f"   📈 Final closure: heading={final_heading_diff:.3f}rad, " +
                  f"curvature={final_curvature_diff:.3f}rad/m, velocity={final_velocity_diff:.3f}m/s")
        else:
            print(
                f"   ⚠️  Large position gap ({position_diff:.3f}m) - trajectory may need track data review")

        return waypoints

    def _validate_trajectory_boundaries(self, waypoints: List[Waypoint]) -> List[Waypoint]:
        """Validate that trajectory waypoints stay within reasonable bounds and don't violate track boundaries."""
        if len(waypoints) < 2:
            return waypoints

        print("   🔍 Validating trajectory boundary compliance...")

        violations_found = 0
        # Maximum reasonable distance between consecutive points
        max_reasonable_distance = 10.0

        validated_waypoints = []

        for i, wp in enumerate(waypoints):
            is_valid = True

            # Check for reasonable coordinate values
            if abs(wp.x_m) > 1000 or abs(wp.y_m) > 1000:
                print(
                    f"   ⚠️  Waypoint {i} has extreme coordinates: ({wp.x_m:.2f}, {wp.y_m:.2f})")
                is_valid = False

            # Check for reasonable velocity and acceleration values
            if wp.vx_mps < 0 or wp.vx_mps > 100:
                print(
                    f"   ⚠️  Waypoint {i} has unreasonable velocity: {wp.vx_mps:.2f} m/s")
                # Clamp velocity to reasonable range
                wp = Waypoint(
                    id=wp.id, s_m=wp.s_m, d_m=wp.d_m, x_m=wp.x_m, y_m=wp.y_m,
                    d_right=wp.d_right, d_left=wp.d_left, psi_rad=wp.psi_rad,
                    kappa_radpm=wp.kappa_radpm, vx_mps=max(
                        0.5, min(50.0, wp.vx_mps)),
                    ax_mps2=wp.ax_mps2
                )

            if abs(wp.ax_mps2) > 20:
                print(
                    f"   ⚠️  Waypoint {i} has extreme acceleration: {wp.ax_mps2:.2f} m/s²")
                # Clamp acceleration to reasonable range
                wp = Waypoint(
                    id=wp.id, s_m=wp.s_m, d_m=wp.d_m, x_m=wp.x_m, y_m=wp.y_m,
                    d_right=wp.d_right, d_left=wp.d_left, psi_rad=wp.psi_rad,
                    kappa_radpm=wp.kappa_radpm, vx_mps=wp.vx_mps,
                    ax_mps2=max(-15.0, min(15.0, wp.ax_mps2))
                )

            # Check for sudden jumps between consecutive waypoints
            if i > 0:
                prev_wp = validated_waypoints[-1]
                distance = math.sqrt((wp.x_m - prev_wp.x_m)
                                     ** 2 + (wp.y_m - prev_wp.y_m)**2)

                if distance > max_reasonable_distance:
                    print(
                        f"   ⚠️  Large jump detected between waypoints {i-1} and {i}: {distance:.2f}m")
                    violations_found += 1
                    is_valid = False

            if is_valid:
                validated_waypoints.append(wp)
            else:
                violations_found += 1

        if violations_found > 0:
            print(
                f"   ⚠️  Found {violations_found} boundary/validity violations")
            print(
                f"   ✅ Validated trajectory reduced to {len(validated_waypoints)} valid waypoints")
        else:
            print(
                f"   ✅ All {len(waypoints)} waypoints passed boundary validation")

        return validated_waypoints if validated_waypoints else waypoints

    def _has_constant_velocity(self, waypoints: List[Waypoint]) -> bool:
        """Check if waypoints have constant velocity profile."""
        if len(waypoints) < 2:
            return True

        velocities = [wp.vx_mps for wp in waypoints]
        velocity_range = max(velocities) - min(velocities)
        return velocity_range < 0.1  # Less than 0.1 m/s variation

    def _apply_velocity_optimization(self, waypoints: List[Waypoint],
                                     optimization_type: str) -> List[Waypoint]:
        """Apply curvature-based velocity optimization."""
        if not waypoints:
            return []

        print(
            f"   🔄 Applying {optimization_type.upper()} velocity optimization based on track curvature...")

        # Load car parameters and trajectory optimization parameters
        car_params = self._load_car_parameters()
        traj_params = self._load_trajectory_optimization_parameters()

        max_velocity = car_params.get("max_velocity", 10.0)
        max_lateral_accel = traj_params["max_lateral_accel_optimization"]
        min_velocity = traj_params["min_velocity"]

        # Adjust parameters based on optimization type
        if optimization_type == OptimizationType.MINTIME:
            lateral_accel_factor = 1.0
            velocity_factor = 1.0
        else:
            lateral_accel_factor = traj_params["lateral_accel_factor_moderate"]
            velocity_factor = traj_params["velocity_factor_moderate"]

        max_lateral_accel *= lateral_accel_factor
        max_velocity *= velocity_factor

        # Calculate velocity limits based on curvature
        optimized_waypoints = []
        for wp in waypoints:
            abs_curvature = abs(wp.kappa_radpm)

            if abs_curvature > 0.001:
                # Calculate maximum safe velocity for this curvature
                v_max_curve = math.sqrt(max_lateral_accel / abs_curvature)
                velocity = min(max_velocity, max(min_velocity, v_max_curve))
            else:
                velocity = max_velocity

            # Create new waypoint with optimized velocity
            new_wp = Waypoint(
                id=wp.id, s_m=wp.s_m, d_m=wp.d_m, x_m=wp.x_m, y_m=wp.y_m,
                d_right=wp.d_right, d_left=wp.d_left, psi_rad=wp.psi_rad,
                kappa_radpm=wp.kappa_radpm, vx_mps=velocity, ax_mps2=wp.ax_mps2
            )
            optimized_waypoints.append(new_wp)

        # Apply smoothing with configurable parameters
        smoothed_waypoints = self._smooth_velocity_profile(
            optimized_waypoints, traj_params)

        # Calculate acceleration profile
        final_waypoints = self._calculate_acceleration_profile(
            smoothed_waypoints)

        # Report results
        velocities = [wp.vx_mps for wp in final_waypoints]
        print(
            f"   ✅ Velocity optimization completed: {min(velocities):.2f} - {max(velocities):.2f} m/s")
        print(
            f"      Max lateral accel: {max_lateral_accel:.1f}m/s², velocity factor: {velocity_factor:.1f}")

        return final_waypoints

    def _smooth_velocity_profile(self, waypoints: List[Waypoint], traj_params: dict = None) -> List[Waypoint]:
        """Apply smoothing to velocity profile."""
        if len(waypoints) < 3:
            return waypoints

        if traj_params is None:
            traj_params = self._load_trajectory_optimization_parameters()

        # Calculate window size using configurable parameters
        window_size = max(
            traj_params["min_smoothing_window"],
            min(traj_params["max_smoothing_window"],
                len(waypoints) // traj_params["velocity_smoothing_window_factor"])
        )
        smoothed_waypoints = []

        for i, wp in enumerate(waypoints):
            # Get window bounds
            start_idx = max(0, i - window_size // 2)
            end_idx = min(len(waypoints), i + window_size // 2 + 1)

            # Calculate smoothed velocity
            window_velocities = [
                waypoints[j].vx_mps for j in range(start_idx, end_idx)]
            smoothed_velocity = sum(window_velocities) / len(window_velocities)

            # Create smoothed waypoint
            smoothed_wp = Waypoint(
                id=wp.id, s_m=wp.s_m, d_m=wp.d_m, x_m=wp.x_m, y_m=wp.y_m,
                d_right=wp.d_right, d_left=wp.d_left, psi_rad=wp.psi_rad,
                kappa_radpm=wp.kappa_radpm, vx_mps=smoothed_velocity, ax_mps2=wp.ax_mps2
            )
            smoothed_waypoints.append(smoothed_wp)

        return smoothed_waypoints

    def _calculate_acceleration_profile(self, waypoints: List[Waypoint]) -> List[Waypoint]:
        """Calculate acceleration based on velocity changes."""
        if len(waypoints) < 2:
            return waypoints

        final_waypoints = []

        for i, wp in enumerate(waypoints):
            if i == 0:
                # First waypoint
                next_wp = waypoints[i + 1]
                ds = abs(next_wp.s_m - wp.s_m) or 1.0
                dv = next_wp.vx_mps - wp.vx_mps
                dt = ds / max(wp.vx_mps, 1.0)
                acceleration = dv / dt if dt > 0 else 0.0
            elif i == len(waypoints) - 1:
                # Last waypoint
                prev_wp = waypoints[i - 1]
                ds = abs(wp.s_m - prev_wp.s_m) or 1.0
                dv = wp.vx_mps - prev_wp.vx_mps
                dt = ds / max(prev_wp.vx_mps, 1.0)
                acceleration = dv / dt if dt > 0 else 0.0
            else:
                # Middle waypoint
                prev_wp = waypoints[i - 1]
                next_wp = waypoints[i + 1]
                ds = abs(next_wp.s_m - prev_wp.s_m) or 2.0
                dv = next_wp.vx_mps - prev_wp.vx_mps
                dt = ds / max(wp.vx_mps, 1.0)
                acceleration = dv / dt if dt > 0 else 0.0

            # Limit acceleration to vehicle capability (3.0 m/s² for NUC2)
            traj_params = self._load_trajectory_optimization_parameters()
            max_accel_limit = traj_params.get("max_longitudinal_accel", 3.0)
            acceleration = max(-max_accel_limit,
                               min(max_accel_limit, acceleration))

            # Create final waypoint
            final_wp = Waypoint(
                id=wp.id, s_m=wp.s_m, d_m=wp.d_m, x_m=wp.x_m, y_m=wp.y_m,
                d_right=wp.d_right, d_left=wp.d_left, psi_rad=wp.psi_rad,
                kappa_radpm=wp.kappa_radpm, vx_mps=wp.vx_mps, ax_mps2=acceleration
            )
            final_waypoints.append(final_wp)

        return final_waypoints

    def _cleanup_temp_files(self, files: List[str]):
        """Clean up temporary files."""
        for filepath in files:
            try:
                if os.path.exists(filepath):
                    os.remove(filepath)
            except Exception as e:
                print(f"Warning: Failed to cleanup {filepath}: {e}")

    def create_trajectories(self, input_data: Dict[str, List[Waypoint]], cache_content) -> Dict[str, List[Waypoint]]:
        print("\n" + "="*80)
        print("🚗 TRAJECTORY GENERATION PIPELINE")
        print("="*80)

        # 3.1: Generate Shortest Path if needed
        trajectory_data = self._generate_shortest_path_if_needed(
            input_data)

        # 3.2: Generate Race Line if needed
        trajectory_data = self._generate_race_line_if_needed(trajectory_data)

        # 3.3: Final summary
        self._print_final_trajectory_summary(trajectory_data)

        # 3.4: Clean up metadata before returning
        cleaned_data = {}
        for key, value in trajectory_data.items():
            # Include waypoint lists and trackbounds data, but not metadata strings
            if not key.endswith('_source') and isinstance(value, list):
                cleaned_data[key] = value
            # Also include trackbounds_markers for unified processing
            elif key == 'trackbounds_markers':
                cleaned_data[key] = value

        return cleaned_data

    def _generate_shortest_path_if_needed(self, input_data: Dict[str, List[Waypoint]]) -> Dict[str, List[Waypoint]]:
        """Generate shortest path trajectory if not cached."""
        print("\n📍 STEP 3.1: SHORTEST PATH PROCESSING")
        print("-" * 50)

        centerline_waypoints = input_data['centerline']
        trackbounds_left = input_data.get('trackbounds_left')
        trackbounds_right = input_data.get('trackbounds_right')

        # Check cache first
        if self.cache_manager:
            cached_sp = self.cache_manager.load_trajectory_cache(
                'shortest_path')
        else:
            cached_sp = None

        if cached_sp:
            print(
                f"💾 CACHE HIT: Loaded {len(cached_sp)} cached shortest path waypoints")
            print("   ✅ Using existing optimized shortest path from cache")
            input_data['sp'] = cached_sp
            input_data['sp_source'] = 'CACHED'
        else:
            print("💾 CACHE MISS: No cached shortest path found")
            print("🚀 INITIATING TRAJECTORY OPTIMIZATION...")
            sp_waypoints = self.generate_shortest_path(
                centerline_waypoints, trackbounds_left, trackbounds_right)

            if sp_waypoints:
                # Only interpolate if the trajectory is significantly different in size
                # Preserve smooth optimization results by avoiding unnecessary downsampling
                size_ratio = len(sp_waypoints) / len(centerline_waypoints)
                if size_ratio < 0.5 or size_ratio > 2.0:  # Only if significantly different
                    print(
                        f"🔧 Interpolating waypoints due to significant size difference: {len(sp_waypoints)} → {len(centerline_waypoints)} (ratio: {size_ratio:.2f})")
                    sp_waypoints = interpolate_waypoints(
                        sp_waypoints, len(centerline_waypoints))
                else:
                    print(
                        f"✅ Preserving optimizer resolution: {len(sp_waypoints)} points (smooth velocity profile maintained)")

                input_data['sp'] = sp_waypoints
                input_data['sp_source'] = 'GENERATED'

                # Cache the result
                if self.cache_manager:
                    self.cache_manager.save_trajectory_cache(
                        'shortest_path', sp_waypoints)
                    print("💾 Saved to cache for future use")

                print(
                    f"✅ SUCCESS: Generated {len(sp_waypoints)} shortest path waypoints")
            else:
                print("❌ GENERATION FAILED: Using centerline as fallback")
                input_data['sp'] = centerline_waypoints
                input_data['sp_source'] = 'FALLBACK_CENTERLINE'

        return input_data

    def _generate_race_line_if_needed(self, input_data: Dict[str, List[Waypoint]]) -> Dict[str, List[Waypoint]]:
        """Generate race line trajectory if not cached."""
        print("\n🏁 STEP 3.2: RACING LINE PROCESSING")
        print("-" * 50)

        centerline_waypoints = input_data['centerline']
        trackbounds_left = input_data.get('trackbounds_left')
        trackbounds_right = input_data.get('trackbounds_right')

        # Check if racing line generation is disabled
        if self.config.racing_line_type == 'disable':
            print("⚠️  RACING LINE DISABLED: Using input data racing line")
            if 'iqp' in input_data:
                print(
                    f"   📥 Using {len(input_data['iqp'])} racing line waypoints from input data")
                input_data['rl_source'] = 'INPUT_DATA'
            else:
                print("   ⚠️  No input racing line found, using centerline")
                input_data['iqp'] = centerline_waypoints
                input_data['rl_source'] = 'FALLBACK_CENTERLINE'
            return input_data

        # Check cache first
        if self.cache_manager:
            cached_rl = self.cache_manager.load_trajectory_cache(
                'racing_line', self.config.racing_line_type, self.config.car_name)
        else:
            cached_rl = None

        if cached_rl:
            print(
                f"💾 CACHE HIT: Loaded {len(cached_rl)} cached racing line waypoints")
            print(
                f"   ✅ Using existing {self.config.racing_line_type.upper()} optimization for {self.config.car_name}")
            input_data['iqp'] = cached_rl
            input_data['rl_source'] = 'CACHED'
        else:
            print("💾 CACHE MISS: No cached racing line found")
            print(
                f"🚀 INITIATING {self.config.racing_line_type.upper()} OPTIMIZATION for {self.config.car_name}")
            racing_line = self.generate_racing_line(
                centerline_waypoints, self.config.racing_line_type,
                trackbounds_left, trackbounds_right)

            if racing_line:
                # Only interpolate if the trajectory is significantly different in size
                # Preserve smooth optimization results by avoiding unnecessary downsampling
                size_ratio = len(racing_line) / len(centerline_waypoints)
                if size_ratio < 0.5 or size_ratio > 2.0:  # Only if significantly different
                    print(
                        f"🔧 Interpolating waypoints due to significant size difference: {len(racing_line)} → {len(centerline_waypoints)} (ratio: {size_ratio:.2f})")
                    racing_line = interpolate_waypoints(
                        racing_line, len(centerline_waypoints))
                else:
                    print(
                        f"✅ Preserving optimizer resolution: {len(racing_line)} points (smooth velocity profile maintained)")

                input_data['iqp'] = racing_line
                input_data['rl_source'] = 'GENERATED'

                # Cache the result
                if self.cache_manager:
                    self.cache_manager.save_trajectory_cache(
                        'racing_line', racing_line,
                        self.config.racing_line_type, self.config.car_name)
                    print("💾 Saved to cache for future use")

                print(
                    f"✅ SUCCESS: Generated {len(racing_line)} racing line waypoints")
            else:
                print("❌ GENERATION FAILED: Using fallback racing line")
                if 'iqp' in input_data:
                    print(
                        f"   📥 Using {len(input_data['iqp'])} racing line waypoints from input data")
                    input_data['rl_source'] = 'FALLBACK_INPUT'
                else:
                    print("   ⚠️  No input racing line available, using centerline")
                    input_data['iqp'] = centerline_waypoints
                    input_data['rl_source'] = 'FALLBACK_CENTERLINE'

        return input_data

    def _print_final_trajectory_summary(self, trajectory_data: Dict[str, List[Waypoint]]):
        """Print a clear summary of how the final trajectory data was derived."""
        print("\n" + "="*80)
        print("📊 FINAL TRAJECTORY SUMMARY")
        print("="*80)

        # Centerline (always from input)
        if 'centerline' in trajectory_data:
            print(
                f"🏁 CENTERLINE:    {len(trajectory_data['centerline']):4d} waypoints  📥 INPUT DATA")

        # Shortest Path
        if 'sp' in trajectory_data:
            sp_source = trajectory_data.get('sp_source', 'UNKNOWN')
            source_icon = {
                'CACHED': '💾',
                'GENERATED': '🔧',
                'FALLBACK_CENTERLINE': '⚠️ ',
                'FALLBACK_INPUT': '🔄'
            }.get(sp_source, '❓')
            print(
                f"📍 SHORTEST PATH: {len(trajectory_data['sp']):4d} waypoints  {source_icon} {sp_source}")

        # Racing Line
        if 'iqp' in trajectory_data:
            rl_source = trajectory_data.get('rl_source', 'UNKNOWN')
            source_icon = {
                'CACHED': '💾',
                'GENERATED': '🔧',
                'INPUT_DATA': '📥',
                'FALLBACK_CENTERLINE': '⚠️ ',
                'FALLBACK_INPUT': '🔄'
            }.get(rl_source, '❓')
            optimization_type = self.config.racing_line_type.upper() if hasattr(self,
                                                                                'config') else 'UNKNOWN'
            print(
                f"🏁 RACING LINE:   {len(trajectory_data['iqp']):4d} waypoints  {source_icon} {rl_source} ({optimization_type})")

        # Trackbounds
        if 'trackbounds_left' in trajectory_data or 'trackbounds_right' in trajectory_data:
            left_count = len(trajectory_data.get('trackbounds_left', []))
            right_count = len(trajectory_data.get('trackbounds_right', []))
            total_count = left_count + right_count
            if total_count > 0:
                print(
                    f"🛤️  TRACKBOUNDS:   {total_count:4d} boundary points ({left_count} left, {right_count} right)  📥 INPUT DATA")

        print("-" * 80)
        print("LEGEND:")
        print("  💾 CACHED           - Loaded from cache (fastest)")
        print("  🔧 GENERATED        - Freshly optimized using TUM trajectory optimizer")
        print("  📥 INPUT_DATA       - Used original data from CSV file")
        print("  🔄 FALLBACK_INPUT   - Generation failed, used input data as backup")
        print("  ⚠️  FALLBACK_CENTERLINE - Used centerline as last resort fallback")
        print("="*80 + "\n")

    def _is_shortest_path_cached(self) -> bool:
        """Check if shortest path is available in cache."""
        if not self.cache_manager:
            return False
        cached_sp = self.cache_manager.load_trajectory_cache('shortest_path')
        return cached_sp is not None

    def _is_race_line_cached(self) -> bool:
        """Check if race line is available in cache."""
        if not self.cache_manager:
            return False
        cached_rl = self.cache_manager.load_trajectory_cache(
            'racing_line', self.config.racing_line_type, self.config.car_name)
        return cached_rl is not None


def interpolate_waypoints(waypoints: List[Waypoint], target_count: int) -> List[Waypoint]:
    """Interpolate waypoints to achieve target count with smooth velocity preservation."""
    if len(waypoints) >= target_count:
        # Instead of simple downsampling, use smart sampling to preserve velocity smoothness
        print(
            f"   ⚠️  Preserving trajectory resolution: keeping {len(waypoints)} points instead of downsampling to {target_count}")
        print(f"   💡 This preserves the smooth velocity profile from the optimizer")
        # Return original trajectory to preserve smooth optimization result
        return waypoints

    try:
        from scipy.interpolate import interp1d, PchipInterpolator

        print(
            f"   🔧 Upsampling trajectory from {len(waypoints)} to {target_count} points with smooth interpolation")

        # Extract arrays for interpolation
        indices = np.arange(len(waypoints))
        x_coords = np.array([wp.x_m for wp in waypoints])
        y_coords = np.array([wp.y_m for wp in waypoints])
        velocities = np.array([wp.vx_mps for wp in waypoints])
        accelerations = np.array([wp.ax_mps2 for wp in waypoints])
        headings = np.array([wp.psi_rad for wp in waypoints])
        curvatures = np.array([wp.kappa_radpm for wp in waypoints])
        s_values = np.array([wp.s_m for wp in waypoints])

        # Create smooth interpolators - use PCHIP for better velocity preservation
        x_interp = PchipInterpolator(indices, x_coords)
        y_interp = PchipInterpolator(indices, y_coords)
        # Monotonic preserving for smooth velocity
        v_interp = PchipInterpolator(indices, velocities)
        a_interp = interp1d(indices, accelerations,
                            kind='linear', fill_value='extrapolate')
        h_interp = interp1d(indices, headings, kind='linear',
                            fill_value='extrapolate')
        k_interp = interp1d(indices, curvatures,
                            kind='linear', fill_value='extrapolate')
        s_interp = PchipInterpolator(indices, s_values)

        # Generate new indices
        new_indices = np.linspace(0, len(waypoints) - 1, target_count)

        # Interpolate with smooth preservation
        interpolated_waypoints = []
        for i, idx in enumerate(new_indices):
            interpolated_wp = Waypoint(
                id=i,
                s_m=float(s_interp(idx)),
                d_m=0.0,  # Optimized trajectory should be on the racing line
                x_m=float(x_interp(idx)),
                y_m=float(y_interp(idx)),
                d_right=1.0,  # Will be updated if needed
                d_left=1.0,   # Will be updated if needed
                psi_rad=float(h_interp(idx)),
                kappa_radpm=float(k_interp(idx)),
                vx_mps=float(v_interp(idx)),  # Smooth velocity interpolation
                ax_mps2=float(a_interp(idx))
            )
            interpolated_waypoints.append(interpolated_wp)

        print(f"   ✅ Smooth interpolation completed - velocity profile preserved")
        return interpolated_waypoints

    except ImportError:
        print("   ⚠️  SciPy not available - using simple linear interpolation")
        # Simple linear interpolation fallback
        interpolated_waypoints = []
        for i in range(target_count):
            ratio = i / (target_count - 1) if target_count > 1 else 0
            idx = ratio * (len(waypoints) - 1)

            # Get surrounding waypoints
            idx_low = int(idx)
            idx_high = min(idx_low + 1, len(waypoints) - 1)
            alpha = idx - idx_low

            wp_low = waypoints[idx_low]
            wp_high = waypoints[idx_high]

            # Linear interpolation
            interpolated_wp = Waypoint(
                id=i,
                s_m=wp_low.s_m + alpha * (wp_high.s_m - wp_low.s_m),
                d_m=wp_low.d_m,
                x_m=wp_low.x_m + alpha * (wp_high.x_m - wp_low.x_m),
                y_m=wp_low.y_m + alpha * (wp_high.y_m - wp_low.y_m),
                d_right=wp_low.d_right,
                d_left=wp_low.d_left,
                psi_rad=wp_low.psi_rad,
                kappa_radpm=wp_low.kappa_radpm,
                vx_mps=wp_low.vx_mps + alpha *
                (wp_high.vx_mps - wp_low.vx_mps),
                ax_mps2=wp_low.ax_mps2
            )
            interpolated_waypoints.append(interpolated_wp)

        return interpolated_waypoints
