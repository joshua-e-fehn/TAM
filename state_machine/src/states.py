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
    if (state_machine.ot_planner == "spliner" or state_machine.ot_planner == "predictive_spliner") and state_machine.last_valid_avoidance_wpnts is not None:
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
