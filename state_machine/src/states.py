from state_machine_node import StateMachine
from f110_msgs.msg import WpntArray, Wpnt
from typing import List

"""
Here we define the behaviour in the different states.
Every function should be fairly concise, and output an array of f110_msgs.Wpnt
"""


def Ready(state_machine: StateMachine) -> List[Wpnt]:
    """READY state - car is stationary, waiting for race start command"""
    # Create simple stationary waypoints to prevent "No waypoints" warnings
    # This keeps the car stationary while providing valid waypoints to the controller

    # Create a simple stationary waypoint
    stationary_waypoint = Wpnt()
    stationary_waypoint.x_m = 0.0
    stationary_waypoint.y_m = 0.0
    stationary_waypoint.psi_rad = 0.0
    stationary_waypoint.kappa_radpm = 0.0
    stationary_waypoint.vx_mps = 0.0  # Zero velocity - keep car stationary
    stationary_waypoint.s_m = 0.0
    stationary_waypoint.d_m = 0.0
    stationary_waypoint.ax_mps2 = 0.0
    stationary_waypoint.d_left = 0.0
    stationary_waypoint.d_right = 0.0
    stationary_waypoint.id = 0

    # Return multiple copies of the same stationary waypoint
    return [stationary_waypoint for _ in range(state_machine.n_loc_wpnts)]


def GlobalTracking(state_machine: StateMachine) -> List[Wpnt]:
    s = int(state_machine.cur_s/state_machine.waypoints_dist + 0.5)
    return [state_machine.glb_wpnts[(s + i) % state_machine.num_glb_wpnts] for i in range(state_machine.n_loc_wpnts)]


def Trailing(state_machine: StateMachine) -> List[Wpnt]:
    # This allows us to trail on the last valid spline if necessary
    if (state_machine.ot_planner in ["spliner", "predictive_spliner", "predictive_sampler"]) and state_machine.last_valid_avoidance_wpnts is not None:
        splini_wpts = state_machine.get_splini_wpts()
        s = int(state_machine.cur_s/state_machine.waypoints_dist + 0.5)
        return [splini_wpts[(s + i) % state_machine.num_glb_wpnts] for i in range(state_machine.n_loc_wpnts)]
    else:
        s = int(state_machine.cur_s/state_machine.waypoints_dist + 0.5)
        return [state_machine.glb_wpnts[(s + i) % state_machine.num_glb_wpnts] for i in range(state_machine.n_loc_wpnts)]


def Overtaking(state_machine: StateMachine) -> List[Wpnt]:
    if (state_machine.ot_planner == "spliner" or state_machine.ot_planner == "predictive_spliner"):
        splini_wpts = state_machine.get_splini_wpts()
        s = int(state_machine.cur_s/state_machine.waypoints_dist + 0.5)
        return [splini_wpts[(s + i) % state_machine.num_glb_wpnts] for i in range(state_machine.n_loc_wpnts)]
    elif state_machine.ot_planner == "predictive_sampler":
        # Use TAM waypoint generation for overtaking (via splini fusion approach)
        tam_wpts = state_machine.get_splini_wpts()
        s = int(state_machine.cur_s / state_machine.waypoints_dist + 0.5)
        return [tam_wpts[(s + i) % state_machine.num_glb_wpnts] for i in range(state_machine.n_loc_wpnts)]
    elif state_machine.ot_planner == "graph_based":
        graph_based_wpnts = state_machine.get_graph_based_wpts()
        return [wpnt for wpnt in graph_based_wpnts.wpnts]
    elif state_machine.ot_planner == "frenet":
        frenet_wpnts = state_machine.frenet_wpnts
        return [wpnt for wpnt in frenet_wpnts.wpnts]
    else:
        s = state_machine.cur_id_ot
        return [state_machine.overtake_wpnts[(s + i) % state_machine.num_ot_points] for i in range(state_machine.n_loc_wpnts)]


def FTGOnly(state_machine: StateMachine):
    """No waypoints are generated in this follow the gap only state, all the control inputs are generated in the control node."""
    return []


def TAMTracking(state_machine: StateMachine) -> List[Wpnt]:
    """
    TAM Sampling planner state - uses trajectories from TAM planner.

    TAM handles all decision-making internally (overtaking, trailing, obstacle avoidance, etc.),
    so the state machine fuses the TAM trajectory with global waypoints and extracts smoothly.

    Uses Spliner-style fusion approach:
    1. Merge TAM trajectory into global waypoint array
    2. Extract waypoints using current s-position (smooth spatial indexing)
    3. No jumping - same extraction pattern as Spliner/Predictive Spliner

    Falls back to global waypoints if no TAM trajectory is available.

    Sets state_machine.tam_waypoint_source attribute for visualization:
    - 'tam_planner': Waypoints from TAM sampling planner
    - 'global_fallback': Waypoints from global raceline fallback
    """
    import rospy
    from std_msgs.msg import String

    # Create publisher for waypoint source (create once, cache in state_machine)
    if not hasattr(state_machine, 'tam_waypoint_source_pub'):
        # Clean the car name to create a valid ROS topic (remove brackets and spaces)
        car_name = state_machine.name.replace(
            '[', '').replace(']', '').replace(' ', '_').lower()
        source_topic = f'/{car_name}/tam_waypoint_source'
        state_machine.tam_waypoint_source_pub = rospy.Publisher(
            source_topic, String, queue_size=1)

    # DEBUG: Log state of avoidance waypoints
    avoidance_status = "None" if state_machine.avoidance_wpnts is None else f"{len(state_machine.avoidance_wpnts.wpnts)} wpnts"
    last_valid_status = "None" if state_machine.last_valid_avoidance_wpnts is None else f"{len(state_machine.last_valid_avoidance_wpnts)} wpnts"
    rospy.loginfo_throttle(2.0,
                           f"[{state_machine.name}] TAMTracking() called | avoidance_wpnts={avoidance_status}, last_valid={last_valid_status}")

    # Check if TAM has published valid waypoints
    if state_machine.avoidance_wpnts and len(state_machine.avoidance_wpnts.wpnts) > 0:
        # Get fused waypoint array (TAM merged with global)
        tam_wpts = state_machine.get_splini_wpts()

        # Extract waypoints using spatial indexing (same as Spliner/GlobalTracking)
        # This ensures smooth progression as car moves - no jumping!
        s = int(state_machine.cur_s / state_machine.waypoints_dist + 0.5)
        selected_wpnts = [tam_wpts[(s + i) % state_machine.num_glb_wpnts]
                          for i in range(state_machine.n_loc_wpnts)]

        # Mark source as TAM planner for visualization
        state_machine.tam_waypoint_source = 'tam_planner'

        # Publish waypoint source
        state_machine.tam_waypoint_source_pub.publish(
            String(data='tam_planner'))

        return selected_wpnts
    else:
        # Fallback: No TAM trajectory available, use global waypoints
        # This happens when TAM planning fails or during initialization
        # rospy.logerr(
        #     f"[{state_machine.name}] ⚠️⚠️⚠️ TAM TRAJECTORY NOT AVAILABLE! --> Fallback to GLOBAL RACELINE waypoints.")

        # Mark source as global fallback for visualization
        state_machine.tam_waypoint_source = 'global_fallback'

        # Publish waypoint source
        state_machine.tam_waypoint_source_pub.publish(
            String(data='global_fallback'))

        # # Use global waypoints starting from current position
        s = int(state_machine.cur_s/state_machine.waypoints_dist + 0.5)
        return [state_machine.glb_wpnts[(s + i) % state_machine.num_glb_wpnts] for i in range(state_machine.n_loc_wpnts)]
