import importlib.util
import math
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "hybrid_target_bridge.py"
SPEC = importlib.util.spec_from_file_location("hybrid_target_bridge", SCRIPT)
BRIDGE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BRIDGE)


def test_camera_world_round_trip():
    point_camera = (1.2, 0.2, 0.0)
    target_world = BRIDGE.measured_tag_to_target_world(
        point_camera,
        0.0, 0.0, 0.0,
        0.35, 0.0, 0.0,
        -0.26, 0.0, 0.0,
        0.0,
    )
    generated = BRIDGE.target_world_to_tag_camera(
        target_world,
        0.0, 0.0, 0.0,
        0.35, 0.0, 0.0,
        -0.26, 0.0, 0.0,
        0.0,
    )
    assert math.isclose(generated[0], point_camera[0], abs_tol=1e-9)
    assert math.isclose(generated[1], point_camera[1], abs_tol=1e-9)


def test_virtual_ego_motion_changes_relative_measurement():
    target_world = (1.81, 0.2)
    generated = BRIDGE.target_world_to_tag_camera(
        target_world,
        0.4, 0.1, 0.2,
        0.35, 0.0, 0.0,
        -0.26, 0.0, 0.0,
        0.0,
    )
    recovered_world = BRIDGE.measured_tag_to_target_world(
        (generated[0], generated[1], 0.0),
        0.4, 0.1, 0.2,
        0.35, 0.0, 0.0,
        -0.26, 0.0, 0.0,
        0.0,
    )
    assert math.isclose(recovered_world[0], target_world[0], abs_tol=1e-9)
    assert math.isclose(recovered_world[1], target_world[1], abs_tol=1e-9)
    assert not math.isclose(generated[0], 1.2)


def test_measured_heading_is_transformed_to_anchor_odom():
    target_world_yaw = BRIDGE.measured_heading_to_world(
        0.20,
        anchor_ego_yaw=0.10,
        camera_yaw=-0.05,
        tag_yaw=0.02,
    )
    assert math.isclose(target_world_yaw, 0.23, abs_tol=1e-9)


def test_measured_heading_wraps_across_pi():
    target_world_yaw = BRIDGE.measured_heading_to_world(
        3.10,
        anchor_ego_yaw=3.05,
        camera_yaw=0.0,
        tag_yaw=0.0,
    )
    assert math.isclose(
        target_world_yaw,
        BRIDGE.wrap_angle(6.15),
        abs_tol=1e-9,
    )


def test_tag_world_position_does_not_depend_on_target_heading():
    point_camera = (1.2, 0.2, 0.0)
    tag_world = BRIDGE.measured_tag_to_world(
        point_camera,
        ego_x=0.0,
        ego_y=0.0,
        ego_yaw=0.0,
        camera_x=0.35,
        camera_y=0.0,
        camera_yaw=0.0,
    )
    assert math.isclose(tag_world[0], 1.55, abs_tol=1e-9)
    assert math.isclose(tag_world[1], 0.2, abs_tol=1e-9)
