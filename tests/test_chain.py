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

    def test_the_tongs_reach_the_wood_at_the_firmware_floor(self):
        """`GROUND_Z_MM` is what pins the tongs' length, and it is not free.

        A floor is the TCP height at which the lowest thing on the arm touches
        the ground. The tong tips are that thing, so tongs of the right length
        put the tips exactly on the desk when the TCP is at `GROUND_Z_MM`. If
        this drifts, either the gripper is the wrong height or the firmware's
        floor has moved and the sim no longer agrees with the machine.
        """
        from mt4_sim.chain import DESK_Z_MM, GROUND_Z_MM, TONG_REACH_MM

        self.assertAlmostEqual(GROUND_Z_MM - TONG_REACH_MM, DESK_Z_MM, places=9)

    def test_the_tongs_clear_the_wood_while_gripping(self):
        """At grip height the tips must be off the desk but well down a cube.

        Tips resting on the wood is the arrangement this replaced: the blades
        dragged, and whichever bit first was held by desk friction, which walked
        the cube across the gripper.
        """
        from mt4_sim.chain import TCP_GRIP_Z_MM, TONG_CLEARANCE_MM, TONG_REACH_MM

        self.assertGreater(TONG_CLEARANCE_MM, 0.0)
        # Still deep enough to hold a 20 mm cube by its sides, not its top edge.
        cube_mm = 20.0
        self.assertGreater(cube_mm - TONG_CLEARANCE_MM, cube_mm / 2.0)
        self.assertAlmostEqual(
            TCP_GRIP_Z_MM - TONG_REACH_MM, TONG_CLEARANCE_MM, places=9
        )

    def test_the_blades_are_as_wide_as_the_real_ones(self):
        from mt4_sim.urdf import FINGER_HEIGHT_MM, FINGER_WIDTH_MM, LINKS

        by_name = {link.name: link for link in LINKS}
        for side in ("finger_left", "finger_right"):
            box = by_name[side].shapes[0]
            self.assertAlmostEqual(box.size[0], FINGER_WIDTH_MM, places=9)
            self.assertAlmostEqual(box.size[2], FINGER_HEIGHT_MM, places=9)

    def test_the_jaw_coupling_is_a_zero_sum_over_the_two_axes(self):
        """The scissor constraint is q_left - q_right = 0, at any opening.

        `finger_positions_for_s` is what the drive is handed, so if it ever
        stopped being symmetric the tendon would be fighting the drive rather
        than the cube.
        """
        for s in (GRIPPER_S_OPEN, 140.0, 180.0, GRIPPER_S_CLOSED):
            left, right = finger_positions_for_s(s)
            self.assertAlmostEqual(1.0 * left + -1.0 * right, 0.0, places=12)

    def test_the_grip_force_is_the_spring_not_the_cap(self):
        """The drive must NOT saturate, and this is why.

        This assertion used to run the other way -- the host closes past contact,
        so the error is large, so the force is always the cap and the cap *is*
        the grip. True, and it costs the jaws their midpoint: two drives clipped
        to the same cap in opposite directions restore the pair's midpoint with
        exactly zero force, and the pair walks out from under the wrist. So the
        cap has to stay clear of everything the spring can ask for, and the grip
        force comes from k times the opening the object leaves.
        """
        from mt4_sim.chain import (
            FINGER_EFFORT_N,
            FINGER_GRIP_ERROR_M,
            FINGER_GRIP_FORCE_N,
            FINGER_STIFFNESS_N_PER_M,
            GRIPPER_S_CLOSED,
            MAX_SPAN_MM,
            finger_positions_for_s,
        )

        # Jaws stopped on a 20 mm cube while the host commands fully shut.
        target = finger_positions_for_s(GRIPPER_S_CLOSED)[0]
        self.assertEqual(target, 0.0)
        spring_n = FINGER_STIFFNESS_N_PER_M * (FINGER_GRIP_ERROR_M - target)
        self.assertAlmostEqual(spring_n, FINGER_GRIP_FORCE_N, places=9)
        self.assertLess(spring_n, FINGER_EFFORT_N)

        # And not at any opening either: a jaw at its open stop is the largest
        # error the drive ever sees, and even that has to stay under the cap.
        widest_n = FINGER_STIFFNESS_N_PER_M * (0.5 * MAX_SPAN_MM * 1e-3)
        self.assertLess(widest_n, FINGER_EFFORT_N)

    def test_grip_force_is_sized_for_the_cube_not_the_solver(self):
        from mt4_sim.chain import (
            FINGER_ARMATURE_KG,
            FINGER_DAMPING_N_S_PER_M,
            FINGER_GRIP_FORCE_N,
            FINGER_STIFFNESS_N_PER_M,
            MAX_SPAN_MM,
        )

        cube_weight_n = 0.008 * 9.81
        # Enough: a friction hold needs N * mu >= mg/2 per pad, mu ~ 1.1, and
        # the arm accelerates the cube as well as carrying it.
        self.assertGreater(FINGER_GRIP_FORCE_N, 10.0 * cube_weight_n)

        # Not too much: the velocity a drive can inject into a 20 g finger in
        # one 240 Hz substep is what launches a gripped cube. At the 12 N cap
        # two designs ago it was 2.5 m/s and cubes were thrown across the desk.
        #
        # Two things in that sum were wrong and are corrected here rather than
        # relaxed. The force is the most the drive can actually deliver, which
        # is no longer FINGER_EFFORT_N -- the cap is deliberately slack and the
        # spring never reaches it -- so it is k over the full travel. And the
        # inertia is the armature, not the bare blade: FINGER_ARMATURE_KG is
        # precisely the mass the drive has to accelerate, which is the argument
        # that fixed the damping derivation in chain.py and never reached this
        # test. Both were left as they were for a design that saturated.
        peak_n = FINGER_STIFFNESS_N_PER_M * (0.5 * MAX_SPAN_MM * 1e-3)
        driven_kg, substep_s = FINGER_ARMATURE_KG + 0.02, 1.0 / 240.0
        self.assertLess(peak_n * substep_s / driven_kg, 0.5)

        # And the damping. This used to assert "at least critically damped",
        # which was only ever true against the bare blade: at the armature-
        # inclusive mass the drive has always been underdamped, and chain.py
        # says outright that zeta > 1 is not reachable here. Asserting it anyway
        # made a false claim pass, so assert the thing that actually bounds c in
        # this design instead.
        #
        # The old bound was a speed floor, F/c, and it is gone with saturation.
        # What replaced it is tracking: a position drive following a ramp lags
        # by c*rate/k, and the firmware advances each jaw at ~0.096 m/s, so too
        # much damping leaves the jaws arriving after the host has moved on.
        from mt4_sim.firmware.state import GRIPPER_SWEEP_RATE_S_PER_S  # noqa: F401

        jaw_sweep_m_s = 0.096
        lag_m = FINGER_DAMPING_N_S_PER_M * jaw_sweep_m_s / FINGER_STIFFNESS_N_PER_M
        self.assertLess(lag_m, 0.005)
        # Underdamped, but not so far that it rings: zeta at the mass the drive
        # actually accelerates.
        zeta = FINGER_DAMPING_N_S_PER_M / (
            2.0 * math.sqrt(FINGER_STIFFNESS_N_PER_M * (FINGER_ARMATURE_KG + 0.02))
        )
        self.assertGreater(zeta, 0.2)

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
