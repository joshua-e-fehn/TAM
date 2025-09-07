#!/usr/bin/env python3
"""
Track scaling utilities for Marina map parser.
"""
import math
from typing import List, Dict, Any, Tuple
from config import Waypoint


class TrackScaler:
    """Handles track scaling operations."""

    def __init__(self, scale_factor: float, csv_file: str = None):
        """Initialize with scale factor and CSV file path."""
        self.scale_factor = scale_factor
        self.csv_file = csv_file

    def scale_waypoint(self, waypoint: Waypoint) -> Waypoint:
        """Scale a single waypoint."""
        return Waypoint(
            id=waypoint.id,
            s_m=waypoint.s_m * self.scale_factor,
            d_m=waypoint.d_m * self.scale_factor,
            x_m=waypoint.x_m * self.scale_factor,
            y_m=waypoint.y_m * self.scale_factor,
            d_right=waypoint.d_right * self.scale_factor,
            d_left=waypoint.d_left * self.scale_factor,
            psi_rad=waypoint.psi_rad,
            kappa_radpm=waypoint.kappa_radpm / self.scale_factor,
            vx_mps=waypoint.vx_mps * self.scale_factor,
            ax_mps2=waypoint.ax_mps2
        )

    def scale_waypoints(self, waypoints: List[Waypoint]) -> List[Waypoint]:
        """Scale a list of waypoints."""
        return [self.scale_waypoint(wp) for wp in waypoints]

    def translate_waypoint(self, waypoint: Waypoint, offset_x: float, offset_y: float) -> Waypoint:
        """Translate a single waypoint by the given offset."""
        return Waypoint(
            id=waypoint.id,
            s_m=waypoint.s_m,
            d_m=waypoint.d_m,
            x_m=waypoint.x_m + offset_x,
            y_m=waypoint.y_m + offset_y,
            d_right=waypoint.d_right,
            d_left=waypoint.d_left,
            psi_rad=waypoint.psi_rad,
            kappa_radpm=waypoint.kappa_radpm,
            vx_mps=waypoint.vx_mps,
            ax_mps2=waypoint.ax_mps2
        )

    def translate_waypoints(self, waypoints: List[Waypoint], offset_x: float, offset_y: float) -> List[Waypoint]:
        """Translate a list of waypoints by the given offset."""
        return [self.translate_waypoint(wp, offset_x, offset_y) for wp in waypoints]

    def scale_and_translate_waypoints(self, waypoints: List[Waypoint]) -> List[Waypoint]:
        """Scale waypoints and translate so that the first waypoint is at origin (0,0)."""
        if not waypoints:
            return []

        # First, scale all waypoints
        scaled_waypoints = self.scale_waypoints(waypoints)

        # Get the position of the first (start) waypoint
        first_waypoint = scaled_waypoints[0]
        offset_x = -first_waypoint.x_m
        offset_y = -first_waypoint.y_m

        # Translate all waypoints so the first one is at origin
        translated_waypoints = self.translate_waypoints(
            scaled_waypoints, offset_x, offset_y)

        print(
            f"✓ Translated track: start point moved from ({first_waypoint.x_m:.6f}, {first_waypoint.y_m:.6f}) to (0.000000, 0.000000)")

        return translated_waypoints

    def scale_trajectories(self, trajectory_data: Dict[str, List[Waypoint]], cache_content: Dict[str, List[Waypoint]]) -> Tuple[Dict[str, List[Waypoint]], Tuple[float, float]]:
        """Simplified method - scaling should be done upfront now."""
        print("Warning: scale_trajectories called but scaling should be done upfront now")
        translation_offset = getattr(
            self, 'last_translation_offset', (0.0, 0.0))
        return trajectory_data, translation_offset

    def scale_trackbounds(self, translation_offset: Tuple[float, float] = None, input_data: Dict[str, List[Waypoint]] = None) -> Dict[str, List[Waypoint]]:
        """Simplified method - trackbounds should be scaled upfront now."""
        print("Warning: scale_trackbounds called but scaling should be done upfront now")
        return {'trackbounds_left': [], 'trackbounds_right': []}

    def scale_map(self, trajectory_data: Dict[str, List[Waypoint]], cache_content: Dict[str, List[Waypoint]], input_data: Dict[str, List[Waypoint]]) -> Tuple[Dict[str, List[Waypoint]], Dict]:
        """
        Simplified method for backward compatibility.
        Since scaling is now done upfront, this just returns the data as-is.
        """
        print("Warning: scale_map called but scaling should be done upfront now")
        # Return empty trackbounds since they should already be in trajectory_data
        empty_trackbounds = {'trackbounds_left': [], 'trackbounds_right': []}
        translation_offset = getattr(
            self, 'last_translation_offset', (0.0, 0.0))
        return trajectory_data, empty_trackbounds, translation_offset


def scale_waypoints(waypoints: List[Waypoint], scale_factor: float) -> List[Waypoint]:
    """Convenience function for scaling waypoints with a given scale factor."""
    scaler = TrackScaler(scale_factor)
    return scaler.scale_waypoints(waypoints)
