#!/usr/bin/env python3
"""
Configuration classes and constants for Marina map parser.
"""
from dataclasses import dataclass
from typing import Dict, Any
import os


@dataclass
class MapConfig:
    """Configuration for map generation."""
    input_file: str  # Can be CSV or JSON file
    base_map_name: str = "marina"
    scale_factor: float = 0.1
    width_multiplier: float = 1.0
    car_name: str = "NUC2"
    racing_line_type: str = "mintime"
    force_regenerate: bool = False
    input_type: str = None  # 'csv' or 'json', auto-detected if None
    # Enable rolling start optimization instead of closed loop
    rolling_start: bool = False
    initial_velocity: float = 5.0  # Initial velocity for rolling start in m/s
    # Vehicle width in meters (from URDF track width)
    car_width: float = 0.2032

    def __post_init__(self):
        """Generate derived properties after initialization."""
        # Auto-detect input type if not specified
        if self.input_type is None:
            self.input_type = self._detect_input_type()

        # Set base map name from filename if not changed from default
        if self.base_map_name == "marina" and self.input_type == "json":
            self.base_map_name = self._extract_map_name_from_json()

        self.output_map_name = self._generate_map_name()
        self.cache_dir = self._setup_cache_dir()

    def _detect_input_type(self) -> str:
        """Detect input file type based on extension."""
        if self.input_file.lower().endswith('.csv'):
            return 'csv'
        elif self.input_file.lower().endswith('.json'):
            return 'json'
        else:
            raise ValueError(
                f"Unsupported input file type: {self.input_file}. Supported types: .csv, .json")

    def _extract_map_name_from_json(self) -> str:
        """Extract map name from JSON file path."""
        import os
        # Look for the map name in the path structure
        path_parts = os.path.normpath(self.input_file).split(os.sep)

        # Try to find map name from parent directory (maps/<mapname>/global_waypoints.json)
        if 'maps' in path_parts:
            maps_index = path_parts.index('maps')
            if maps_index + 1 < len(path_parts):
                return path_parts[maps_index + 1]

        # Fallback to filename without extension
        filename = os.path.basename(self.input_file)
        return os.path.splitext(filename)[0].replace('global_waypoints', 'map')

    def _generate_map_name(self) -> str:
        """Generate map name based on parameters."""
        size_percent = int(self.scale_factor * 100)
        width_percent = int(self.width_multiplier * 100)
        return f"{self.base_map_name}_{size_percent}%s_{width_percent}%w_{self.car_name}_{self.racing_line_type}"

    def _setup_cache_dir(self) -> str:
        """Set up cache directory."""
        cache_dir = os.path.join(os.path.dirname(self.input_file), "cache")
        os.makedirs(cache_dir, exist_ok=True)
        return cache_dir

    # Legacy property for backward compatibility
    @property
    def csv_file(self) -> str:
        """Legacy property for backward compatibility."""
        return self.input_file


@dataclass
class Waypoint:
    """Represents a single waypoint with all necessary properties."""
    id: int
    s_m: float  # Arc length
    d_m: float  # Lateral offset
    x_m: float  # X coordinate
    y_m: float  # Y coordinate
    d_right: float  # Distance to right boundary
    d_left: float   # Distance to left boundary
    psi_rad: float  # Heading angle
    kappa_radpm: float  # Curvature
    vx_mps: float   # Velocity
    ax_mps2: float  # Acceleration

    def to_dict(self) -> Dict[str, Any]:
        """Convert waypoint to dictionary format."""
        return {
            'id': self.id,
            's_m': self.s_m,
            'd_m': self.d_m,
            'x_m': self.x_m,
            'y_m': self.y_m,
            'd_right': self.d_right,
            'd_left': self.d_left,
            'psi_rad': self.psi_rad,
            'kappa_radpm': self.kappa_radpm,
            'vx_mps': self.vx_mps,
            'ax_mps2': self.ax_mps2
        }


class CSVColumnMapping:
    """Column mapping for Marina CSV format."""
    MAPPING = {
        'rl_x_m': 0, 'rl_y_m': 1, 'rl_vx_mps': 3, 'rl_psi_rad': 5,
        'rl_ax_mps2': 6, 'rl_n_m': 4, 'ref_rl_s_m': 11, 'ref_rl_x_m': 12,
        'ref_rl_y_m': 13, 'ref_rl_psi_rad': 15, 'ref_rl_kappa_radpm': 18,
        'ref_rl_d_right': 21, 'ref_rl_d_left': 22, 'ref_cl_s_m': 26,
        'ref_cl_x_m': 27, 'ref_cl_y_m': 28, 'ref_cl_psi_rad': 30,
        'ref_cl_kappa_radpm': 33, 'ref_cl_d_right': 36, 'ref_cl_d_left': 37,
        'tb_left_x': 41, 'tb_left_y': 42, 'tb_right_x': 44, 'tb_right_y': 45,
    }


class TrajectoryType:
    """Constants for trajectory types."""
    CENTERLINE = "centerline"
    IQP = "iqp"
    SP = "sp"
    RACING_LINE = "racing_line"
    SHORTEST_PATH = "shortest_path"


class OptimizationType:
    """Constants for optimization types."""
    MINTIME = "mintime"
    MINCURV = "mincurv"
    DISABLE = "disable"


# Trajectory optimization parameters for different vehicle configurations
TRAJECTORY_OPTIMIZATION_PARAMS = {
    "DEFAULT": {
        # Vehicle dynamics parameters - Updated for NUC2
        "mass": 3.54,  # kg - from NUC2 car_model.yaml
        "wheelbase": 0.307,  # m - from NUC2 car_model.yaml
        "lf": 0.162,  # m - front axle distance from CG
        "lr": 0.145,  # m - rear axle distance from CG
        "Iz": 0.05797,  # kg*m^2 - moment of inertia
        "h_cg": 0.014,  # m - height of center of gravity
        "dragcoeff": 0.05,
        "curvlim": 0.4189,  # max steering angle from NUC2 config
        "wheel_radius": 0.0325,
        "gravity": 9.81,

        # Tire parameters - Updated for NUC2 Pacejka model
        "C_Sf": 4.798521440254997,  # Front cornering stiffness from NUC2_pacejka.yaml
        "C_Sr": 19.999999999999996,  # Rear cornering stiffness from NUC2_pacejka.yaml
        "lam_muy_f": 0.8,
        "lam_muy_r": 0.8,
        "muy": 1.0,  # Friction coefficient from NUC2_pacejka.yaml
        "camber": 0.0,
        "c_roll": 0.013,
        "f_z0": 150.0,  # F1Tenth scale normal force
        "B_front": 10.0,
        "C_front": 2.1640281784621833,  # From NUC2 Pacejka parameters
        "eps_front": 0.6502296853018044,
        "E_front": 0.3732212044732381,
        "B_rear": 10.0,
        "C_rear": 1.4999999999999998,  # From NUC2 Pacejka parameters
        "eps_rear": 0.6184183350099146,
        "E_rear": 1.1322308905491715e-16,

        # Powertrain parameters - Adjusted for NUC2
        "power_max": 15000.0,  # Watts
        "f_drive_max": 1000.0,  # N
        "f_brake_max": 1500.0,  # N
        "motor_efficiency": 0.85,
        "max_acceleration": 2.5,  # m/s^2 from vesc.yaml

        # Vehicle dynamics coefficients
        "liftcoeff_front": 0.0,
        "liftcoeff_rear": 0.0,
        "k_brake_front": 0.6,
        "k_drive_front": 0.0,
        "k_roll": 0.5,

        # Time constants - Updated from NUC2 config
        "t_delta": 0.15779476,  # tau_steer from NUC2 config
        "t_drive": 0.05,
        "t_brake": 0.05,
        "max_servo_speed": 3.2,  # rad/s from vesc.yaml

        # Vehicle dimensions
        "car_width": 0.2032,  # Vehicle width in meters (from URDF track width)

        # Optimization parameters
        "safety_width": 0.5,  # Conservative for NUC2
        # Enable rolling start (True) or closed loop optimization (False)
        "rolling_start": False,
        "initial_velocity": 3.0,  # Initial velocity for rolling start in m/s
        "penalty_delta": 10.0,
        "penalty_F": 0.01,
        "friction_coeff": 1.0,

        # GGV parameters - Adjusted for NUC2 capabilities
        "max_longitudinal_accel": 3.0,  # a_max from NUC2 car_model.yaml
        "max_lateral_accel": 6.0,  # Reduced for more conservative NUC2
        "accel_speed_factor": 20.0,  # Speed factor for acceleration reduction
        "lateral_speed_factor": 25.0,  # Speed factor for lateral acceleration reduction
        "min_accel": 2.0,

        # Power curve parameters
        # Maximum acceleration from power curve (match a_max)
        "power_curve_max_accel": 3.0,
        "power_curve_factor": 50.0,  # Power factor for acceleration curve
        "friction_limited_accel": 3.0,  # Reduced to match NUC2 a_max

        # Velocity optimization parameters - Adjusted for NUC2
        "max_lateral_accel_optimization": 6.0,  # Conservative for NUC2
        "min_velocity": 1.0,  # Lower minimum for NUC2
        "max_velocity": 10.0,  # v_max from NUC2 car_model.yaml
        "lateral_accel_factor_moderate": 0.8,
        "velocity_factor_moderate": 0.9,

        # Smoothing parameters
        "velocity_smoothing_window_factor": 100,  # Waypoint count / this factor
        "min_smoothing_window": 3,
        "max_smoothing_window": 15,

        # Spline parameters - optimized for NUC2
        "stepsize_prep": 0.5,  # Good resolution for trajectory prep
        "stepsize_reg": 1.5,   # Smooth speed profiles for NUC2
        "stepsize_interp_after_opt": 1.0,  # Smooth interpolation
        "k_reg": 3,
        "s_reg": 10,
        "d_preview_curv": 2.0,
        "d_review_curv": 2.0,
        "d_preview_head": 1.0,
        "d_review_head": 1.0,

        # Velocity calculation parameters
        # No filtering window (null in TUM config)
        "vel_profile_conv_filt_window": None,
        "dyn_model_exp": 1.0,  # Dynamic model exponent for velocity calculation
    },

    "NUC2": {
        # NUC2-specific configuration - inherits from DEFAULT and overrides specific values
        # Vehicle dynamics parameters - NUC2 specific
        "mass": 3.54,  # kg
        "wheelbase": 0.307,  # m
        "lf": 0.162,  # m - front axle distance from CG
        "lr": 0.145,  # m - rear axle distance from CG
        "Iz": 0.05797,  # kg*m^2 - moment of inertia
        "h_cg": 0.014,  # m - height of center of gravity
        "curvlim": 0.4189,  # max steering angle
        "wheel_radius": 0.0325,

        # Tire parameters - NUC2 Pacejka model
        "C_Sf": 4.798521440254997,
        "C_Sr": 19.999999999999996,
        "muy": 1.0,
        "C_front": 2.1640281784621833,
        "eps_front": 0.6502296853018044,
        "E_front": 0.3732212044732381,
        "C_rear": 1.4999999999999998,
        "eps_rear": 0.6184183350099146,
        "E_rear": 1.1322308905491715e-16,

        # Performance limits - NUC2 specific
        "max_longitudinal_accel": 3.0,  # a_max from car_model.yaml
        "max_lateral_accel": 6.0,  # Conservative for stability
        "max_acceleration": 2.5,  # from vesc.yaml
        "max_velocity": 10.0,  # v_max from car_model.yaml
        "min_velocity": 1.0,

        # Time constants - NUC2 specific
        "t_delta": 0.15779476,  # tau_steer
        "max_servo_speed": 3.2,  # rad/s from vesc.yaml

        # Vehicle dimensions - NUC2 specific
        "car_width": 0.2032,  # Vehicle width in meters (from URDF track width)

        # Optimization parameters tuned for NUC2
        "safety_width": 0.5,  # Slightly tighter for NUC2
        "friction_limited_accel": 3.0,
        "max_lateral_accel_optimization": 6.0,
    }
}


def get_trajectory_params(car_name: str = "DEFAULT") -> Dict[str, Any]:
    """
    Get trajectory optimization parameters for a specific car.

    Args:
        car_name: Name of the car configuration (e.g., "NUC2", "DEFAULT")

    Returns:
        Dictionary containing trajectory optimization parameters
    """
    # Start with default parameters
    params = TRAJECTORY_OPTIMIZATION_PARAMS["DEFAULT"].copy()

    # Override with car-specific parameters if available
    if car_name in TRAJECTORY_OPTIMIZATION_PARAMS:
        params.update(TRAJECTORY_OPTIMIZATION_PARAMS[car_name])

    return params
