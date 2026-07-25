from math import isclose
from drone_ground_station_2d.models import VehicleState, quaternion_to_euler_deg, yaw_deg_to_quaternion


def test_yaw_roundtrip():
    q = yaw_deg_to_quaternion(90.0)
    _, _, yaw = quaternion_to_euler_deg(*q)
    assert isclose(yaw, 90.0, abs_tol=1e-6)


def test_goal_error():
    state = VehicleState('x', 'X')
    state.position = [0.0, 0.0, 0.0]
    state.goal = [3.0, 4.0, 0.0]
    state.has_goal = True
    assert isclose(state.goal_error(), 5.0)
