#!/usr/bin/env python3
"""
TAM Sampling Visualization Tool

This script helps visualize what the TAM sampling planner is generating.
It subscribes to debug topics and plots:
- Longitudinal velocity profiles
- Lateral trajectory paths
- Track boundaries
- Valid vs invalid trajectories

Usage:
    rosrun tam_sampling_planner visualize_tam_sampling.py
"""

import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from f110_msgs.msg import WpntArray
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray


class TAMSamplingVisualizer:
    def __init__(self):
        rospy.init_node('tam_sampling_visualizer')

        # Data storage
        self.global_waypoints = None
        self.current_position = None
        self.tam_trajectories = None
        self.all_sampled_trajectories = None  # NEW: Store all sampled trajectories

        # Create figure with subplots
        self.fig, self.axes = plt.subplots(2, 2, figsize=(15, 12))
        self.fig.suptitle('TAM Sampling Planner Visualization', fontsize=16)

        # Subscribe to topics
        self.global_wp_sub = rospy.Subscriber(
            '/car1/global_waypoints', WpntArray, self.global_waypoints_callback)
        self.state_sub = rospy.Subscriber(
            '/car1/car_state/odom_frenet', Odometry, self.state_callback)
        self.markers_sub = rospy.Subscriber(
            '/car1/planner/avoidance/markers', MarkerArray, self.markers_callback)

        # NEW: Subscribe to all sampled trajectories
        self.all_samples_sub = rospy.Subscriber(
            '/car1/planner/avoidance/all_samples', MarkerArray, self.all_samples_callback)

        rospy.loginfo("TAM Sampling Visualizer started. Waiting for data...")

    def global_waypoints_callback(self, msg):
        """Store global waypoints"""
        if len(msg.wpnts) > 0:
            self.global_waypoints = msg
            rospy.loginfo_once("Received global waypoints")

    def state_callback(self, msg):
        """Store current vehicle state"""
        self.current_position = {
            's': msg.pose.pose.position.x,
            'n': msg.pose.pose.position.y,
            'vel': msg.twist.twist.linear.x
        }

    def markers_callback(self, msg):
        """Store TAM trajectory markers"""
        self.tam_trajectories = msg
        rospy.loginfo_throttle(5.0, "Received TAM trajectory markers")

    def all_samples_callback(self, msg):
        """Store all sampled trajectory markers"""
        self.all_sampled_trajectories = msg
        if len(msg.markers) > 0:
            rospy.loginfo_throttle(
                5.0, f"Received {len(msg.markers)} sampled trajectory markers")
        else:
            rospy.logwarn_throttle(5.0, "Received empty all_samples message")

    def plot_track(self, ax):
        """Plot track boundaries from global waypoints"""
        if self.global_waypoints is None:
            return

        x = np.array([wp.x_m for wp in self.global_waypoints.wpnts])
        y = np.array([wp.y_m for wp in self.global_waypoints.wpnts])

        # Get track boundaries
        try:
            d_left = np.array([wp.d_left if hasattr(
                wp, 'd_left') else 1.5 for wp in self.global_waypoints.wpnts])
            d_right = np.array([wp.d_right if hasattr(
                wp, 'd_right') else -1.5 for wp in self.global_waypoints.wpnts])

            # Calculate track heading from consecutive points
            dx = np.diff(x, append=x[-1] - x[-2] + x[-1])
            dy = np.diff(y, append=y[-1] - y[-2] + y[-1])
            heading = np.arctan2(dy, dx)

            # Calculate normal vectors (perpendicular to heading)
            normal_x = -np.sin(heading)
            normal_y = np.cos(heading)

            # Calculate left boundary (positive offset in normal direction)
            x_left = x + d_left * normal_x
            y_left = y + d_left * normal_y

            # Calculate right boundary (d_right is not negative, so substract it)
            x_right = x - d_right * normal_x
            y_right = y - d_right * normal_y

            # Plot centerline
            ax.plot(x, y, 'k--', linewidth=1, label='Centerline', alpha=0.5)

            # Plot boundaries
            ax.plot(x_left, y_left, 'r-', linewidth=2,
                    label='Left boundary', alpha=0.6)
            ax.plot(x_right, y_right, 'b-', linewidth=2,
                    label='Right boundary', alpha=0.6)

            # Fill area between boundaries
            # Concatenate right boundary reversed to create closed polygon
            boundary_x = np.concatenate([x_left, x_right[::-1]])
            boundary_y = np.concatenate([y_left, y_right[::-1]])
            ax.fill(boundary_x, boundary_y, color='gray',
                    alpha=0.1, label='Track area')

        except Exception as e:
            rospy.logwarn(f"Could not plot track boundaries: {e}")
            import traceback
            traceback.print_exc()
            ax.plot(x, y, 'k-', linewidth=1, label='Centerline')

        # Plot ALL sampled trajectories (colored by validity: blue=valid, red=invalid)
        if self.all_sampled_trajectories and len(self.all_sampled_trajectories.markers) > 0:
            rospy.loginfo_throttle(
                10.0, f"Plotting {len(self.all_sampled_trajectories.markers)} sampled trajectories")
            plotted_count = 0
            valid_count = 0
            invalid_count = 0

            for marker in self.all_sampled_trajectories.markers:
                if marker.type == 4:  # LINE_STRIP
                    x_traj = [p.x for p in marker.points]
                    y_traj = [p.y for p in marker.points]
                    if len(x_traj) > 0:
                        # Use marker color to determine if valid (blue) or invalid (red)
                        # Valid trajectories have high blue component (b=1.0)
                        # Invalid trajectories have high red component (r=1.0)
                        is_valid = marker.color.b > 0.5  # Blue channel > 0.5 means valid

                        if is_valid:
                            ax.plot(x_traj, y_traj, color='blue',
                                    alpha=0.1, linewidth=0.5)
                            valid_count += 1
                        else:
                            ax.plot(x_traj, y_traj, color='red',
                                    alpha=0.1, linewidth=0.5)
                            invalid_count += 1
                        plotted_count += 1

            rospy.loginfo_throttle(
                10.0, f"Actually plotted {plotted_count} trajectories (valid={valid_count}, invalid={invalid_count})")

            # Add legend entries for sampled trajectories
            ax.plot([], [], color='blue', alpha=0.3, linewidth=1,
                    label=f'Valid samples ({valid_count})')
            ax.plot([], [], color='red', alpha=0.3, linewidth=1,
                    label=f'Invalid samples ({invalid_count})')
        else:
            if self.all_sampled_trajectories is None:
                rospy.logwarn_throttle(
                    10.0, "all_sampled_trajectories is None")
            else:
                rospy.logwarn_throttle(
                    10.0, f"all_sampled_trajectories has {len(self.all_sampled_trajectories.markers)} markers")

        # Plot selected trajectory on top
        if self.tam_trajectories and len(self.tam_trajectories.markers) > 0:
            for marker in self.tam_trajectories.markers:
                if marker.type == 4:  # LINE_STRIP
                    x_traj = [p.x for p in marker.points]
                    y_traj = [p.y for p in marker.points]
                    ax.plot(x_traj, y_traj, 'g-', linewidth=2,
                            label='Selected trajectory')
                    break  # Only plot first one

        # Plot current vehicle position (estimate from Frenet to global)
        if self.current_position and self.global_waypoints:
            try:
                # Find closest waypoint to current s position
                s_coords = np.array([wp.s_m if hasattr(wp, 's_m') else 0.0
                                    for wp in self.global_waypoints.wpnts])
                idx = np.argmin(np.abs(s_coords - self.current_position['s']))

                wp = self.global_waypoints.wpnts[idx]

                # Get heading at this point
                if idx < len(self.global_waypoints.wpnts) - 1:
                    wp_next = self.global_waypoints.wpnts[idx + 1]
                    heading = np.arctan2(
                        wp_next.y_m - wp.y_m, wp_next.x_m - wp.x_m)
                else:
                    wp_prev = self.global_waypoints.wpnts[idx - 1]
                    heading = np.arctan2(
                        wp.y_m - wp_prev.y_m, wp.x_m - wp_prev.x_m)

                # Calculate normal direction
                normal_x = -np.sin(heading)
                normal_y = np.cos(heading)

                # Estimate global position (centerline + lateral offset)
                car_x = wp.x_m + self.current_position['n'] * normal_x
                car_y = wp.y_m + self.current_position['n'] * normal_y

                # Plot car position
                ax.plot(car_x, car_y, 'go', markersize=12, label='Current car position',
                        zorder=20, markeredgecolor='darkgreen', markeredgewidth=2)

                # Add direction arrow
                arrow_len = 0.3
                ax.arrow(car_x, car_y, arrow_len * np.cos(heading), arrow_len * np.sin(heading),
                         head_width=0.15, head_length=0.1, fc='green', ec='darkgreen',
                         linewidth=2, zorder=21)

            except Exception as e:
                rospy.logwarn_throttle(
                    5.0, f"Could not plot car position: {e}")

        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_title('Track Layout with All Sampled Trajectories')
        ax.legend(loc='best', fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.axis('equal')

    def plot_frenet_trajectories(self, ax):
        """Plot trajectories in Frenet frame (s-n space)"""
        if self.global_waypoints is None:
            return

        # Plot track boundaries in Frenet
        s = np.array([wp.s_m if hasattr(wp, 's_m') else i*0.1 for i,
                      wp in enumerate(self.global_waypoints.wpnts)])

        try:
            d_left = np.array([wp.d_left if hasattr(
                wp, 'd_left') else 1.5 for wp in self.global_waypoints.wpnts])
            d_right = np.array([wp.d_right if hasattr(
                wp, 'd_right') else -1.5 for wp in self.global_waypoints.wpnts])

            # Fill track boundaries
            ax.fill_between(s, d_right, d_left, alpha=0.15,
                            color='gray', label='Track bounds')

            # Plot boundary lines
            ax.plot(s, d_left, 'r-', linewidth=1.5,
                    alpha=0.6, label='Left boundary')
            ax.plot(s, d_right, 'b-', linewidth=1.5,
                    alpha=0.6, label='Right boundary')

            # Plot centerline
            ax.plot(s, [0]*len(s), 'k--', linewidth=1,
                    label='Centerline', alpha=0.5)

        except Exception as e:
            rospy.logwarn(f"Could not plot Frenet boundaries: {e}")

        # Plot current position
        if self.current_position:
            ax.plot(self.current_position['s'], self.current_position['n'],
                    'go', markersize=10, label='Current position', zorder=10)

            # Add text annotation for current position
            ax.annotate(f"Car (s={self.current_position['s']:.1f}m, n={self.current_position['n']:.2f}m)",
                        xy=(self.current_position['s'],
                            self.current_position['n']),
                        xytext=(10, 10), textcoords='offset points',
                        bbox=dict(boxstyle='round,pad=0.5',
                                  fc='yellow', alpha=0.7),
                        fontsize=8)

        ax.set_xlabel('s [m]')
        ax.set_ylabel('n [m]')
        ax.set_title('Frenet Trajectories (s-n space)')
        ax.legend(loc='best', fontsize=8)
        ax.grid(True, alpha=0.3)

    def plot_velocity_profiles(self, ax):
        """Plot velocity profiles over time"""
        if self.global_waypoints is None:
            return

        s = [wp.s_m if hasattr(wp, 's_m') else i*0.1 for i,
             wp in enumerate(self.global_waypoints.wpnts)]
        v = [wp.vx_mps if hasattr(
            wp, 'vx_mps') else 5.0 for wp in self.global_waypoints.wpnts]

        ax.plot(s, v, 'b-', linewidth=2, label='Reference velocity')

        if self.current_position:
            ax.plot(self.current_position['s'], self.current_position['vel'],
                    'go', markersize=10, label='Current velocity')

        ax.set_xlabel('s [m]')
        ax.set_ylabel('Velocity [m/s]')
        ax.set_title('Velocity Profile')
        ax.legend()
        ax.grid(True, alpha=0.3)

    def plot_sampling_stats(self, ax):
        """Plot sampling statistics"""
        ax.clear()

        info_text = "TAM Sampling Statistics\n\n"

        if self.global_waypoints:
            info_text += f"Global waypoints: {len(self.global_waypoints.wpnts)}\n"

        if self.current_position:
            info_text += f"\nCurrent State:\n"
            info_text += f"  s = {self.current_position['s']:.2f} m\n"
            info_text += f"  n = {self.current_position['n']:.2f} m\n"
            info_text += f"  v = {self.current_position['vel']:.2f} m/s\n"

        # NEW: Show sampling statistics
        if self.all_sampled_trajectories:
            info_text += f"\nTrajectory Sampling:\n"
            info_text += f"  Total sampled: {len(self.all_sampled_trajectories.markers)}\n"

        if self.tam_trajectories:
            info_text += f"  Selected: 1 trajectory\n"

        # Check ROS parameters
        try:
            info_text += f"\nSampling Parameters:\n"
            info_text += f"  lateral_samples = {rospy.get_param('~lateral_samples', 'N/A')}\n"
            info_text += f"  n_dense_samples = {rospy.get_param('~n_dense_samples', 'N/A')}\n"
            info_text += f"  tube_width = {rospy.get_param('behavior/tube_width', 'N/A'):.2f} m\n"
            info_text += f"  safety_left = {rospy.get_param('safety_distances/safety_distance_track_left', 'N/A'):.2f} m\n"
            info_text += f"  safety_right = {rospy.get_param('safety_distances/safety_distance_track_right', 'N/A'):.2f} m\n"
            info_text += f"  vehicle_width = {rospy.get_param('width', 'N/A'):.2f} m\n"
        except:
            pass

        ax.text(0.1, 0.5, info_text, fontsize=10, verticalalignment='center',
                family='monospace', transform=ax.transAxes)
        ax.axis('off')

    def update(self, frame):
        """Update all plots"""
        for ax in self.axes.flat:
            ax.clear()

        self.plot_track(self.axes[0, 0])
        self.plot_frenet_trajectories(self.axes[0, 1])
        self.plot_velocity_profiles(self.axes[1, 0])
        self.plot_sampling_stats(self.axes[1, 1])

        plt.tight_layout()

    def run(self):
        """Run the visualizer"""
        # Wait for initial data
        rospy.sleep(1.0)

        # Set up animation
        ani = FuncAnimation(self.fig, self.update,
                            interval=1000, cache_frame_data=False)

        plt.show()

        # Keep running until shutdown
        rospy.spin()


if __name__ == '__main__':
    try:
        viz = TAMSamplingVisualizer()
        viz.run()
    except rospy.ROSInterruptException:
        pass
