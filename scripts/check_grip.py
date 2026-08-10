"""Close on a cube, lift it, and assert the grip is a grip and not a shove.

Headless physics check for grip plausibility. The failure this exists to catch
is not "the cube fell out" -- it is the force fight going wrong in either
direction: jaws so weak an 8 g cube shoves them open, or so strong the solver
launches the cube across the desk. So it measures, through the close:

* how far each jaw ended up, and how *unequal* the two are -- a lopsided pair
  means one jaw is pressing on the cube while the other is not, which slides it
  out sideways rather than holding it;
* how fast the cube ever moved, which is the direct reading of "thrown about";
* how far it drifted in XY, which is the same thing integrated;
* whether it rotated into face alignment, which is what the grip force is for.

    python scripts/check_grip.py [--cube N] [--yaw-error DEG]

``--yaw-error`` deliberately mis-aims the wrist so the jaws have to rotate the
cube into line. That is the grip force doing useful work, and it is the reason
the force is not simply set as low as it will go.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from mt4_sim.paths import SCENE_USD  # noqa: E402

from isaacsim import SimulationApp  # noqa: E402

_args = argparse.ArgumentParser(description=__doc__)
_args.add_argument("--cube", type=int, default=1, help="1-based index into rig.CUBES")
_args.add_argument(
    "--yaw-error",
    type=float,
    default=0.0,
    help="degrees to mis-aim the wrist, so the jaws must rotate the cube",
)
ARGS = _args.parse_args()

app = SimulationApp({"headless": True})

from pxr import Usd, UsdPhysics, UsdGeom, UsdShade  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.stage import get_current_stage, open_stage  # noqa: E402

from mt4_sim import rig  # noqa: E402
from mt4_sim.arm import (  # noqa: E402
    SimArm,
    bind_finger_friction,
    link_prim_path,
    prepare_gripper_contacts,
)
from mt4_sim.firmware.state import CART_SEGMENT_MM  # noqa: E402
from mt4_sim.scene import physics_dt  # noqa: E402
from mt4_sim.chain import (  # noqa: E402
    TCP_GRIP_Z_MM,
    FINGER_ARMATURE_KG,
    FINGER_DAMPING_N_S_PER_M,
    FINGER_EFFORT_N,
    FINGER_GRIP_FORCE_N,
    FINGER_JOINT_NAMES,
    FINGER_STIFFNESS_N_PER_M,
    FINGER_WINDUP_M,
    MAX_SPAN_MM,
    GRIPPER_S_CLOSED,
    GRIPPER_S_OPEN,
    JointAnglesDeg,
    MM,
    park_pose,
    span_mm_for_s,
)
from mt4_jog.kinematics import ik_position, ws_j4_deg  # noqa: E402

LIFT_MM = 40.0
CUBE_KG = 0.008
FINGER_KG = 0.02
SUBSTEP_S = 1.0 / 240.0

# What "good" means here. The jaws are *supposed* to shove a misaligned cube --
# that is how a real gripper squares one up, and it cannot happen without the
# cube moving fast and far. So this does not police motion during the close; it
# polices where the cube ends up.
MAX_CUBE_SPEED_M_S = 1.5  # a squaring cube reaches ~0.6; past this it is launched
# Cube centre versus the TCP once the jaws are shut. The jaws are coupled but
# deliberately softly (see `chain.FINGER_COUPLING_STIFFNESS`), so whichever
# reaches the cube first still walks it some way across the gripper before the
# other arrives; past ~16 mm a 20 mm cube has less than half a blade under it
# and the grasp is not one you would trust through a move. The worst mis-aim in
# this set lands at 2.9 mm.
GRIP_CAPTURE_MM = 16.0
GAP_TOLERANCE_MM = 2.0  # how far the gap may sit from the cube's own span
# How far the wrist can be mis-aimed and still have the jaws pull the cube
# square. Measured: 12 deg snaps flat during the lift, 15 and 20 stay stably
# corner-gripped -- which is a real grasp, just not an aligned one, so beyond
# this the check asks only that the cube is carried.
ALIGNS_UP_TO_DEG = 12.0


def _with_yaw(q: JointAnglesDeg, yaw_deg: float, x_mm: float, y_mm: float) -> JointAnglesDeg:
    """Aim the wrist at a cube's face. ``yaw_deg`` is the cube's robot-frame yaw.

    ``j4_for_face_align`` answers in the **world** frame -- ``ws_j4_deg`` is
    ``j4 + j1`` -- while ``JointAnglesDeg.j4`` is the model angle the firmware
    speaks. Taking the answer as a model angle leaves the jaws off by j1, which
    at a cube out at (232, -95) is 22 deg: enough that the jaws close on the
    cube's corners at a 26 mm gap instead of its faces at 20 mm.
    """
    try:
        from mt4_vision.wrist import j4_for_face_align

        world_j4 = j4_for_face_align(
            yaw_deg, current_j4_deg=ws_j4_deg(q), x=x_mm, y=y_mm
        )
    except Exception as exc:  # noqa: BLE001 - diagnostic fallback for headless checks
        print(f"face-align unavailable ({exc}); using the cube's own yaw")
        world_j4 = yaw_deg
    return JointAnglesDeg(q.j1, q.j2, q.j3, world_j4 + ARGS.yaw_error - q.j1)


def _cube_prim(stage, index: int):
    cube = rig.CUBES[index - 1]
    path = f"/World/Cubes/cube_{index}_{cube.color}"
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        raise RuntimeError(f"missing cube prim {path}")
    return prim, cube


def _pose_mm(prim) -> tuple[tuple[float, float, float], float]:
    """Cube centre in mm and its yaw about Z in degrees, from the scene graph."""
    mat = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
    t = mat.ExtractTranslation()
    # The cube is a scaled unit cube, so row 0 is its own +X axis in world.
    x_axis = mat.GetRow3(0)
    yaw = math.degrees(math.atan2(x_axis[1], x_axis[0]))
    return (t[0] / MM, t[1] / MM, t[2] / MM), yaw


def _yaw_error_deg(measured: float, wanted: float) -> float:
    """Smallest angle between two yaws, modulo the cube's 90 deg symmetry."""
    return abs((measured - wanted + 45.0) % 90.0 - 45.0)


def _span_across_jaws_mm(off_axis_deg: float) -> float:
    """How wide a cube presents to jaws it is ``off_axis_deg`` away from square.

    Square on, that is the cube's own width; turned, the jaws meet two opposite
    corners and the cube is genuinely wider between them. At 20 deg a 20 mm
    cube spans 25.6 mm, so a fixed "must be about 20" would call a perfectly
    good corner grip a failure.
    """
    theta = math.radians(off_axis_deg)
    return rig.CUBE_SIZE_MM * (math.cos(theta) + math.sin(theta))


def _drive_report(stage) -> None:
    for name in FINGER_JOINT_NAMES:
        prim = stage.GetPrimAtPath(f"/World/MT4/Physics/{name}")
        drive = UsdPhysics.DriveAPI.Get(prim, "linear")
        print(
            f"  {name}: k={drive.GetStiffnessAttr().Get()} "
            f"c={drive.GetDampingAttr().Get()} "
            f"Fmax={drive.GetMaxForceAttr().Get()}"
        )
    # The inertia the drive works against is the armature, not the blade's own
    # mass -- that is what armature means. Sizing zeta off FINGER_KG alone
    # reports 1.68 for a drive that is actually at 0.42, which is the error that
    # let the jaws ring; see FINGER_DAMPING_N_S_PER_M.
    effective_kg = FINGER_KG + FINGER_ARMATURE_KG
    critical = 2.0 * math.sqrt(FINGER_STIFFNESS_N_PER_M * effective_kg)
    # The grip is the servo's torque limit, not the drive's cap and no longer
    # k times an error that depends on the object. Reporting the cap here once
    # read "6.0 N" three lines above a measured press of 1.49 N, which is the
    # kind of number that gets quoted later. The cap is still worth printing --
    # as the headroom it is, because a drive that reaches it loses the midpoint.
    grip_n = FINGER_GRIP_FORCE_N
    print(
        f"  grip force = the servo's torque limit = {grip_n:.2f} N on anything "
        f"more than {2000 * FINGER_WINDUP_M:.1f} mm inside the commanded opening; "
        f"cube weight = {CUBE_KG * 9.81:.3f} N ({grip_n / (CUBE_KG * 9.81):.0f}x)"
    )
    print(
        f"  wind-up {1000 * FINGER_WINDUP_M:.2f} mm -> the blades cannot sweep faster "
        f"than {FINGER_WINDUP_M / (1.0 / 60.0):.3f} m/s (firmware asks 0.096)"
    )
    print(
        f"  backstop cap {FINGER_EFFORT_N:.1f} N vs the {grip_n:.2f} N the position "
        f"loop can ask for -> {1000 * grip_n * SUBSTEP_S / effective_kg:.1f} mm/s of "
        f"kick per substep"
    )
    print(
        f"  damping zeta = {FINGER_DAMPING_N_S_PER_M / critical:.2f}; "
        f"the rate is fed forward, so this costs no tracking"
    )


def _bound_physics_material(prim):
    """The physics-purpose material actually bound to ``prim``, or None."""
    binding = UsdShade.MaterialBindingAPI(prim).GetDirectBinding(materialPurpose="physics")
    target = binding.GetMaterial()
    return target.GetPrim() if target else None


def _friction_report(stage, label: str, prims) -> int:
    """Report bound friction. Reads the binding, not attributes on the collider.

    Attributes written straight onto a collider prim are invisible to PhysX, so
    reading them back would confirm nothing -- this follows the binding.
    """
    n = 0
    for prim in prims:
        material = _bound_physics_material(prim)
        if material is None:
            print(f"  MISSING physics material binding on {prim.GetPath()}")
            continue
        api = UsdPhysics.MaterialAPI(material)
        print(
            f"  {prim.GetPath()} -> {material.GetPath()}: "
            f"static_mu={api.GetStaticFrictionAttr().Get()} "
            f"dynamic_mu={api.GetDynamicFrictionAttr().Get()}"
        )
        n += 1
    print(f"  {label}: {n} bound")
    return n


def _finger_colliders(stage):
    for link in ("finger_left", "finger_right"):
        root = stage.GetPrimAtPath(link_prim_path(link))
        for prim in Usd.PrimRange(root):
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                yield prim


def main() -> int:
    if not SCENE_USD.is_file():
        print(f"missing scene {SCENE_USD}; run scripts/build_scene.py first")
        return 1

    open_stage(str(SCENE_USD))
    stage = get_current_stage()
    # Contact attrs must be finite before the articulation view parses physics.
    prepare_gripper_contacts(stage)
    world = World(stage_units_in_meters=1.0, physics_dt=physics_dt())
    world.reset()
    prepare_gripper_contacts(stage)
    bind_finger_friction(stage)
    arm = SimArm()
    stage = arm._art.prim.GetStage()

    print(f"Physics dt: {world.get_physics_dt() * 1000:.2f} ms")
    print("Finger drive (runtime):")
    _drive_report(stage)

    print("Finger pad friction:")
    if _friction_report(stage, "finger colliders", _finger_colliders(stage)) < 2:
        print("FAIL: both finger colliders need a bound physics material")
        app.close()
        return 1

    cube_prim, cube = _cube_prim(stage, ARGS.cube)
    print("Cube friction:")
    if _friction_report(stage, "cube", [cube_prim]) < 1:
        print("FAIL: the cube needs a bound physics material")
        app.close()
        return 1

    (x0, y0, z0), yaw0 = _pose_mm(cube_prim)
    print(
        f"Target cube {cube.color} #{ARGS.cube} at ({x0:.1f}, {y0:.1f}, {z0:.1f}), "
        f"yaw {yaw0:.1f} deg; wrist mis-aimed by {ARGS.yaw_error:.1f} deg",
        flush=True,
    )

    near = park_pose()
    arm.set_gripper_s(GRIPPER_S_OPEN, teleport=True)

    hover_raw = ik_position(cube.x_mm, cube.y_mm, TCP_GRIP_Z_MM + LIFT_MM, near=near)
    grip_raw = ik_position(cube.x_mm, cube.y_mm, TCP_GRIP_Z_MM, near=hover_raw or near)
    if hover_raw is None or grip_raw is None:
        print("FAIL: IK missed the hover or grip pose")
        app.close()
        return 1
    hover = _with_yaw(hover_raw, cube.yaw_deg, cube.x_mm, cube.y_mm)
    grip = _with_yaw(grip_raw, cube.yaw_deg, cube.x_mm, cube.y_mm)

    arm.set_model_angles(hover, teleport=True)
    for _ in range(30):
        world.step(render=False)
    arm.set_model_angles(grip, teleport=True)
    for _ in range(60):
        world.step(render=False)
    print(f"at grip, jaws at {arm.finger_positions()[0]*1e3:.2f}/"
          f"{arm.finger_positions()[1]*1e3:.2f} mm", flush=True)

    # -- the close ---------------------------------------------------------
    dt = world.get_physics_dt()
    previous, _ = _pose_mm(cube_prim)
    peak_speed = 0.0
    for _ in range(360):
        arm.set_gripper_s(GRIPPER_S_CLOSED)
        world.step(render=False)
        now, _ = _pose_mm(cube_prim)
        step_mm = math.dist(now, previous)
        peak_speed = max(peak_speed, step_mm * MM / dt)
        previous = now

    left, right = arm.finger_positions()
    gap_mm = (left + right) / MM
    asymmetry = abs(left - right) / MM
    (x1, y1, _z1), yaw1 = _pose_mm(cube_prim)
    drift = math.dist((x1, y1), (x0, y0))
    print(
        f"After close: jaws {left*1e3:.2f}/{right*1e3:.2f} mm "
        f"(clear gap {gap_mm:.2f} mm, asymmetry {asymmetry:.2f} mm); "
        f"commanded closed span {span_mm_for_s(GRIPPER_S_CLOSED):.2f} mm; "
        f"pressing at {arm.grip_force_n():.2f} N"
    )
    tcp_now = arm.state().tcp_mm
    captured = math.dist((x1, y1), (tcp_now[0], tcp_now[1]))
    print(
        f"  cube: shoved {drift:.2f} mm in XY, peak speed {peak_speed:.3f} m/s, "
        f"yaw {yaw0:.1f} -> {yaw1:.1f} deg, ended {captured:.2f} mm from the TCP"
    )

    # What the gap *should* be: a cube held at an angle to the jaws is gripped
    # corner to corner and genuinely spans more than its own width. Comparing
    # against a flat 20 mm would fail every deliberately mis-aimed grip.
    residual = _yaw_error_deg(yaw1, ws_j4_deg(grip))
    want_gap = _span_across_jaws_mm(residual)
    print(
        f"  expected gap for a cube {residual:.1f} deg off the jaws: "
        f"{want_gap:.2f} mm"
    )

    failures: list[str] = []
    if gap_mm > want_gap + GAP_TOLERANCE_MM:
        failures.append(
            f"jaws lost the force fight (clear gap {gap_mm:.1f} mm, "
            f"expected {want_gap:.1f})"
        )
    if gap_mm < want_gap - GAP_TOLERANCE_MM:
        failures.append(
            f"jaws crushed into the cube (clear gap {gap_mm:.1f} mm, "
            f"expected {want_gap:.1f})"
        )
    if captured > GRIP_CAPTURE_MM:
        failures.append(
            f"cube ended {captured:.1f} mm from the TCP (> {GRIP_CAPTURE_MM}): "
            f"the leading jaw pushed it out of the gripper instead of onto it"
        )
    if peak_speed > MAX_CUBE_SPEED_M_S:
        failures.append(
            f"cube was launched, not gripped (peak {peak_speed:.2f} m/s "
            f"> {MAX_CUBE_SPEED_M_S})"
        )
    if failures:
        for line in failures:
            print(f"FAIL: {line}")
        app.close()
        return 1

    # -- the lift ----------------------------------------------------------
    lift_raw = ik_position(cube.x_mm, cube.y_mm, TCP_GRIP_Z_MM + LIFT_MM, near=grip)
    if lift_raw is None:
        print("FAIL: IK missed the lift")
        app.close()
        return 1
    lift = _with_yaw(lift_raw, cube.yaw_deg, cube.x_mm, cube.y_mm)
    # Walk the TCP up one `CART_SEGMENT_MM` per step, which is the fastest the
    # firmware can lift: `mp`/`mq` chop a straight world line into 2 mm segments
    # and solve each, so 2 mm per 60 Hz tick is its ceiling.
    #
    # Handing the drives the whole 40 mm at once instead is not a hard version
    # of this test, it is a different one. Measured: the TCP covers 122 -> 161
    # mm in five steps, touching 0.42 m/s, and the cube's own inertia levers the
    # jaws open -- gap 19.7 -> 26.0 mm -- and throws it out. Nothing the host
    # can send produces that acceleration.
    for step in range(240):
        arm.set_gripper_s(GRIPPER_S_CLOSED)
        height = min(LIFT_MM, CART_SEGMENT_MM * (step + 1))
        rung = ik_position(cube.x_mm, cube.y_mm, TCP_GRIP_Z_MM + height, near=grip)
        arm.set_model_angles(_with_yaw(rung, cube.yaw_deg, cube.x_mm, cube.y_mm))
        world.step(render=False)

    (_x2, _y2, z2), yaw2 = _pose_mm(cube_prim)
    tcp = arm.state().tcp_mm
    left, right = arm.finger_positions()
    lift_gap_mm = (left + right) / MM
    off_the_jaws = _yaw_error_deg(yaw2, ws_j4_deg(lift))
    print(
        f"After lift: cube z={z2:.1f} mm (rose {z2 - z0:.1f}), TCP z={tcp[2]:.1f}, "
        f"clear gap {lift_gap_mm:.2f} mm, yaw {yaw2:.1f} deg "
        f"({off_the_jaws:.1f} off the jaws)"
    )

    if z2 - z0 < 15.0:
        print(f"FAIL: cube did not lift with the gripper (rose {z2 - z0:.1f} mm)")
        app.close()
        return 1
    # "Opened under load" is the gap *growing* while carrying, not the gap
    # being wider than a face grip -- a corner-gripped cube is legitimately
    # wider, and one that rotates flat mid-lift legitimately gets narrower.
    if lift_gap_mm > gap_mm + GAP_TOLERANCE_MM:
        print(
            f"FAIL: jaws opened under load ({gap_mm:.1f} -> {lift_gap_mm:.1f} mm)"
        )
        app.close()
        return 1
    if abs(ARGS.yaw_error) <= ALIGNS_UP_TO_DEG and off_the_jaws > 8.0:
        print(
            f"FAIL: jaws did not rotate the cube into line "
            f"({off_the_jaws:.1f} deg off)"
        )
        app.close()
        return 1

    print("PASS: the cube is gripped, carried, and never thrown")
    app.close()
    return 0


if __name__ == "__main__":
    try:
        code = main()
    except Exception:
        app.close()
        raise
    sys.exit(code)
