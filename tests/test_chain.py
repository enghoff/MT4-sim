"""The URDF chain must reproduce the control repo's kinematics exactly.

These are pure maths and need neither Isaac Sim nor a GPU: they compose the
transforms straight out of ``mt4_sim.urdf.JOINTS`` and compare against
``mt4_jog.kinematics``. A sign flip or a wrong joint origin fails here in
milliseconds instead of showing up as a millimetre of pick error in the sim.

``scripts/check.py`` is the complementary test: it checks that the *imported
articulation* behaves like this chain under physics.

    python -m unittest discover -s tests
"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path
from xml.etree import ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np

from mt4_sim.chain import (
    ARM_JOINT_NAMES,
    GRIPPER_S_CLOSED,
    GRIPPER_S_OPEN,
    MM,
    Q_LIMITS,
    finger_positions_for_s,
    model_from_urdf,
    park_pose,
    s_for_span_mm,
    span_mm_for_s,
    urdf_from_model,
)
from mt4_sim.urdf import JOINTS, LINKS, build_urdf
from mt4_jog.joints import JOINT_SOFT_MAX_STEPS, JOINT_SOFT_MIN_STEPS
from mt4_jog.kinematics import JointAnglesDeg, fk_tcp

# Poses across the soft-limit box. Reach is irrelevant here -- this is geometry,
# not physics, so a pose below the desk is a perfectly good test case.
SWEEP = [
    park_pose(),
    JointAnglesDeg(0.0, 60.0, -40.0, 0.0),
    JointAnglesDeg(45.0, 90.0, 10.0, 30.0),
    JointAnglesDeg(-60.0, 30.0, -60.0, -45.0),
    JointAnglesDeg(120.0, 130.0, 20.0, 90.0),
    JointAnglesDeg(-137.0, 22.8, -67.8, -179.0),
    JointAnglesDeg(130.0, 135.5, 23.5, 179.0),
]


def _axis_angle(axis: tuple[float, float, float], angle: float) -> np.ndarray:
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    r = np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * (k @ k)
    out = np.eye(4)
    out[:3, :3] = r
    return out


def _translate(xyz_mm: tuple[float, float, float]) -> np.ndarray:
    out = np.eye(4)
    out[:3, 3] = [v * MM for v in xyz_mm]
    return out


def chain_tcp_mm(q: JointAnglesDeg) -> np.ndarray:
    """TCP by composing the URDF's own joint table, in millimetres."""
    positions = dict(zip(ARM_JOINT_NAMES, urdf_from_model(q)))
    joints = {j.name: j for j in JOINTS}

    transform = np.eye(4)
    for name in ARM_JOINT_NAMES:
        joint = joints[name]
        transform = transform @ _translate(joint.origin_mm) @ _axis_angle(
            joint.axis, positions[name]
        )
    return transform[:3, 3] / MM


class ChainMatchesKinematics(unittest.TestCase):
    def test_tcp_matches_fk_over_the_soft_limit_box(self):
        for q in SWEEP:
            with self.subTest(q=q):
                expected = fk_tcp(q)
                got = chain_tcp_mm(q)
                for axis, want in zip("xyz", (expected.x, expected.y, expected.z)):
                    self.assertAlmostEqual(
                        got["xyz".index(axis)], want, places=9, msg=f"{axis} at {q}"
                    )

    def test_park_pose_reports_the_documented_tcp(self):
        """The firmware's tape-fit park pose: FK (190.0, 0, 225.6)."""
        tcp = chain_tcp_mm(park_pose())
        self.assertAlmostEqual(tcp[0], 190.0, places=1)
        self.assertAlmostEqual(tcp[1], 0.0, places=6)
        self.assertAlmostEqual(tcp[2], 225.6, places=1)

    def test_wrist_roll_does_not_move_the_tcp(self):
        """The roll axis passes through the TCP, which is why fk_tcp has no q4."""
        base = chain_tcp_mm(JointAnglesDeg(20.0, 90.0, -10.0, 0.0))
        for j4 in (-170.0, -45.0, 90.0, 179.0):
            rolled = chain_tcp_mm(JointAnglesDeg(20.0, 90.0, -10.0, j4))
            np.testing.assert_allclose(rolled, base, atol=1e-9)

    def test_head_stays_level_however_the_arm_folds(self):
        """The link rods' job: the head platform's orientation is pose-invariant."""
        joints = {j.name: j for j in JOINTS}
        reference = None
        for q in SWEEP:
            positions = dict(zip(ARM_JOINT_NAMES, urdf_from_model(q)))
            transform = np.eye(4)
            for name in ("j1_base_yaw", "j2_shoulder", "j3_elbow", "j_head_level"):
                joint = joints[name]
                transform = transform @ _translate(joint.origin_mm) @ _axis_angle(
                    joint.axis, positions[name]
                )
            # Undo the base yaw: what must be invariant is the head's tilt out of
            # the arm's own vertical plane, not its heading.
            tilt = _axis_angle((0, 0, 1), -positions["j1_base_yaw"])[:3, :3] @ transform[:3, :3]
            if reference is None:
                reference = tilt
            np.testing.assert_allclose(tilt, reference, atol=1e-9)


class ModelUrdfRoundTrip(unittest.TestCase):
    def test_round_trip(self):
        for q in SWEEP:
            with self.subTest(q=q):
                back = model_from_urdf(urdf_from_model(q))
                for name in ("j1", "j2", "j3", "j4"):
                    self.assertAlmostEqual(getattr(back, name), getattr(q, name), places=9)


class JointLimitsFollowTheFirmware(unittest.TestCase):
    def test_limits_come_from_the_firmware_step_box(self):
        """Each q limit is one end of the firmware's soft step range."""
        from mt4_jog.kinematics import STEPS_PER_DEG, J_STEP_SIGN

        for index, limit in enumerate(Q_LIMITS):
            span_deg = (
                abs(JOINT_SOFT_MAX_STEPS[index] - JOINT_SOFT_MIN_STEPS[index])
                / STEPS_PER_DEG[index]
            )
            self.assertAlmostEqual(limit.upper_deg - limit.lower_deg, span_deg, places=6)
            self.assertEqual(J_STEP_SIGN[index] in (1.0, -1.0), True)

    def test_park_pose_is_inside_the_limits(self):
        q = park_pose()
        for name, limit in zip(("j1", "j2", "j3", "j4"), Q_LIMITS):
            value = getattr(q, name)
            self.assertGreaterEqual(value, limit.lower_deg)
            self.assertLessEqual(value, limit.upper_deg)


class GripperSpanModel(unittest.TestCase):
    def test_span_is_monotonic_and_clamped(self):
        self.assertAlmostEqual(span_mm_for_s(GRIPPER_S_CLOSED), 0.0, places=9)
        self.assertGreater(span_mm_for_s(GRIPPER_S_OPEN), 30.0)
        previous = None
        for s in range(GRIPPER_S_OPEN, GRIPPER_S_CLOSED + 1):
            span = span_mm_for_s(s)
            if previous is not None:
                self.assertLessEqual(span, previous)
            previous = span

    def test_span_round_trips_inside_the_physical_range(self):
        for span in (0.0, 5.0, 20.0, 35.0):
            self.assertAlmostEqual(span_mm_for_s(s_for_span_mm(span)), span, places=6)

    def test_fingers_split_the_span(self):
        left, right = finger_positions_for_s(GRIPPER_S_OPEN)
        self.assertAlmostEqual(left, right, places=12)
        self.assertAlmostEqual(
            (left + right) / MM, span_mm_for_s(GRIPPER_S_OPEN), places=9
        )

    def test_finger_boxes_put_the_inner_face_on_the_joint(self):
        """Clear opening must equal the span model, not span minus blade thickness."""
        from mt4_sim.urdf import FINGER_INNER_TO_CENTER_MM, FINGER_THICK_MM, LINKS

        by_name = {link.name: link for link in LINKS}
        left = by_name["finger_left"].shapes[0]
        right = by_name["finger_right"].shapes[0]
        self.assertAlmostEqual(left.size[1], FINGER_THICK_MM, places=9)
        self.assertAlmostEqual(right.size[1], FINGER_THICK_MM, places=9)
        # Left opens +Y: box grows outward from the inner face at y=0.
        self.assertAlmostEqual(left.centre[1], FINGER_INNER_TO_CENTER_MM, places=9)
        # Right opens -Y: box grows the other way.
        self.assertAlmostEqual(right.centre[1], -FINGER_INNER_TO_CENTER_MM, places=9)

        # At the calibrated open S the clear gap is the span, with room for a cube.
        open_s = 140.0  # grip_open_s on the live calib
        clear_mm = span_mm_for_s(open_s)
        self.assertGreater(clear_mm, 20.0 + 10.0)  # cube + comfortable margin
        self.assertAlmostEqual(clear_mm, 38.4, delta=0.2)

    def test_the_grip_force_is_the_cap_not_the_spring(self):
        """The host closes past contact, so the drive always saturates.

        That is the whole grip model: whatever the object's width, the position
        error against it is large and the jaws press with FINGER_EFFORT_N.
        """
        from mt4_sim.chain import (
            FINGER_EFFORT_N,
            FINGER_STIFFNESS_N_PER_M,
            GRIPPER_S_CLOSED,
            finger_positions_for_s,
        )

        # Jaws stopped on a 20 mm cube while the host commands fully shut.
        target = finger_positions_for_s(GRIPPER_S_CLOSED)[0]
        self.assertEqual(target, 0.0)
        spring_n = FINGER_STIFFNESS_N_PER_M * (0.010 - target)
        self.assertGreater(spring_n, FINGER_EFFORT_N)

    def test_grip_force_is_sized_for_the_cube_not_the_solver(self):
        from mt4_sim.chain import (
            FINGER_DAMPING_N_S_PER_M,
            FINGER_EFFORT_N,
            FINGER_STIFFNESS_N_PER_M,
        )

        cube_weight_n = 0.008 * 9.81
        # Enough: a friction hold needs N * mu >= mg/2 per pad, mu ~ 1.1, and
        # the arm accelerates the cube as well as carrying it.
        self.assertGreater(FINGER_EFFORT_N, 10.0 * cube_weight_n)
        # Not too much: the velocity a capped drive can inject into a 20 g
        # finger in one 240 Hz substep is what launches a gripped cube. At the
        # 12 N this replaces it was 2.5 m/s, and cubes were thrown across the
        # desk.
        finger_kg, substep_s = 0.02, 1.0 / 240.0
        self.assertLess(FINGER_EFFORT_N * substep_s / finger_kg, 0.5)

        # And the drive must settle rather than ring: at least critically damped.
        critical = 2.0 * math.sqrt(FINGER_STIFFNESS_N_PER_M * finger_kg)
        self.assertGreaterEqual(FINGER_DAMPING_N_S_PER_M, critical)

    def test_jaws_close_faster_than_the_firmware_sweeps(self):
        """Terminal closing speed under the force cap is F/c.

        If that were slower than the rate the firmware advances S, the jaws
        would lag their own command through a free-space close and arrive after
        the host has already moved on.
        """
        from mt4_sim.chain import (
            FINGER_DAMPING_N_S_PER_M,
            FINGER_EFFORT_N,
            GRIPPER_S_OPEN,
            span_mm_for_s,
        )
        from mt4_sim.firmware.state import GRIPPER_SWEEP_RATE_S_PER_S

        terminal_m_s = FINGER_EFFORT_N / FINGER_DAMPING_N_S_PER_M
        # Each jaw covers half the span while S crosses the range that moves it.
        moving_s = span_mm_for_s(GRIPPER_S_OPEN) * 1.881
        jaw_m_s = 0.5 * span_mm_for_s(GRIPPER_S_OPEN) * MM / (
            moving_s / GRIPPER_SWEEP_RATE_S_PER_S
        )
        self.assertGreater(terminal_m_s, jaw_m_s)


class UrdfIsWellFormed(unittest.TestCase):
    def setUp(self):
        self.robot = build_urdf().getroot()

    def test_one_inertial_per_link(self):
        for link in self.robot.findall("link"):
            self.assertEqual(len(link.findall("inertial")), 1, link.get("name"))

    def test_masses_match_the_link_table(self):
        by_name = {link.name: link.mass for link in LINKS}
        for link in self.robot.findall("link"):
            mass = float(link.find("inertial/mass").get("value"))
            self.assertAlmostEqual(mass, by_name[link.get("name")], places=6)

    def test_joints_form_one_tree_rooted_at_base_link(self):
        links = {link.get("name") for link in self.robot.findall("link")}
        children = []
        for joint in self.robot.findall("joint"):
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            self.assertIn(parent, links)
            self.assertIn(child, links)
            children.append(child)

        self.assertEqual(len(children), len(set(children)), "a link has two parents")
        roots = links - set(children)
        self.assertEqual(roots, {"base_link"})

    def test_every_joint_has_a_finite_limit(self):
        for joint in self.robot.findall("joint"):
            limit = joint.find("limit")
            lower, upper = float(limit.get("lower")), float(limit.get("upper"))
            self.assertLess(lower, upper, joint.get("name"))
            self.assertGreater(float(limit.get("effort")), 0.0)
            self.assertGreater(float(limit.get("velocity")), 0.0)

    def test_it_is_parseable_xml(self):
        ET.tostring(self.robot)


if __name__ == "__main__":
    unittest.main()
