"""Open the scene, drive the arm, and check the simulation against the real one.

Three things get verified, each of which would silently produce a wrong sim:

1. **The chain matches the kinematic model.** The TCP frame's measured position
   on the stage is compared against ``mt4_jog.kinematics.fk_tcp`` over a sweep
   of poses. A mismatch means the URDF's joint origins or axis signs are wrong,
   and every coordinate the sim produces would be off.
2. **The physics drives reach the pose they are told to.** Commanded model
   angles are compared against the joint positions after stepping.
3. **The simulated camera is usable by the real vision stack.** The scene camera
   is rendered and the frame handed to the same ArUco detector
   ``mt4_vision.detect`` uses. Tags that do not decode make the sim useless for
   anything downstream of calibration.

Writes the rendered frames next to the scene as ``out/``.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from mt4_sim.paths import SCENE_USD  # noqa: E402

OUT = ROOT / "out"

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "renderer": "RaytracedLighting"})

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.stage import open_stage  # noqa: E402

from mt4_sim import rig  # noqa: E402
from mt4_sim.arm import SimArm  # noqa: E402
from mt4_sim.scene import physics_dt  # noqa: E402
from mt4_sim.chain import (  # noqa: E402
    GRIPPER_S_OPEN,
    HEAD_OFFSET,
    JointAnglesDeg,
    park_pose,
)
from mt4_jog.kinematics import fk_tcp  # noqa: E402

# Poses spanning the soft-limit box, so a sign error on any axis shows up. All
# of them keep the TCP above the desk: a pose that puts it below lands the arm
# on the tabletop collider, and the measured pose then reflects the contact
# rather than the chain.
SWEEP: tuple[JointAnglesDeg, ...] = (
    park_pose(),
    JointAnglesDeg(0.0, 60.0, 5.0, 0.0),
    JointAnglesDeg(45.0, 90.0, 10.0, 30.0),
    JointAnglesDeg(-60.0, 70.0, -10.0, -45.0),
    JointAnglesDeg(120.0, 130.0, 20.0, 90.0),
    JointAnglesDeg(-120.0, 100.0, 15.0, -170.0),
)

# At rest the simulated TCP can differ from `fk_tcp` in exactly one way: the
# head platform is held level by a drive rather than by the real arm's link
# rods, so it tilts by the drive's steady-state error and swings the TCP through
# HEAD_OFFSET. The check below asserts the residual *is* that -- which tests the
# chain exactly -- and separately caps how much tilt is tolerated.
#
# UNEXPLAINED_TOLERANCE_MM is the one that tests the chain, and it is met with
# ~100x to spare (0.0002 mm). The other two only bound the tilt stand-in, and
# they scale with the physics step: an implicit position drive is effectively
# softer at a coarser step, so the same drive stiffness droops further. Measured
# at the worst pose in the sweep, tilt and its TCP artefact are
#
#     240 Hz   0.09 deg, 0.05 mm
#      60 Hz   0.18 deg, 0.11 mm
#
# and `mt4_sim.scene.PHYSICS_HZ` is 60. Raising `import_urdf.ARM_STIFFNESS`
# would buy it back if anything ever cares about a tenth of a millimetre.
TCP_TOLERANCE_MM = 0.15
HEAD_TILT_TOLERANCE_DEG = 0.25
UNEXPLAINED_TOLERANCE_MM = 0.02
DRIVE_TOLERANCE_DEG = 0.5

# A tag read out of a simulated frame with the real rig's calibration file
# cannot land exactly: the sim camera is a pinhole and the rig's is not. The
# camera fit's own residual is ~14mm, so this is that plus room for detection
# noise -- tight enough that a misplaced or mis-turned tag still fails.
TAG_PLACEMENT_TOLERANCE_MM = 35.0
TAG_YAW_TOLERANCE_DEG = 5.0


def step(world: World, count: int) -> None:
    for _ in range(count):
        world.step(render=False)


def check_chain(world: World, arm: SimArm) -> list[str]:
    """Measured TCP vs the control repo's FK of the pose the arm is *actually* in.

    Running FK on the *commanded* angles would fold two different errors into one
    number: a wrong URDF chain, and the fraction of a degree a position drive
    gives up to gravity. Only the first makes the sim wrong, so FK is evaluated
    at the measured joint angles. Droop is reported separately as ``hold``.
    """
    failures = []
    print("\n-- chain vs mt4_jog.kinematics.fk_tcp -------------------------------")
    print(
        f"{'q1':>7} {'q2':>7} {'q3':>7} {'q4':>7}   {'fk at measured q (mm)':>26} "
        f"{'chain':>8} {'hold':>7} {'tilt':>7} {'unexpl':>8}"
    )
    for q in SWEEP:
        arm.set_model_angles(q, teleport=True)
        # Settle first. The joint positions come from the physics tensor view and
        # the TCP transform from the scene graph, which is written a step behind,
        # so comparing them while the arm is still moving measures that lag
        # rather than the chain.
        step(world, 90)

        state = arm.state()
        expected = fk_tcp(state.q)
        measured = arm.measured_tcp_mm()
        error = max(
            abs(measured[0] - expected.x),
            abs(measured[1] - expected.y),
            abs(measured[2] - expected.z),
        )
        droop = max(
            abs(getattr(q, n) - getattr(state.q, n)) for n in ("j1", "j2", "j3", "j4")
        )
        tilt = arm.head_tilt_deg()
        # What a head tilted by `tilt` does to a TCP offset HEAD_OFFSET out.
        from_tilt = HEAD_OFFSET * abs(math.sin(math.radians(tilt)))
        unexplained = abs(error - from_tilt)

        bad = (
            error > TCP_TOLERANCE_MM
            or abs(tilt) > HEAD_TILT_TOLERANCE_DEG
            or unexplained > UNEXPLAINED_TOLERANCE_MM
        )
        print(
            f"{q.j1:7.1f} {q.j2:7.1f} {q.j3:7.1f} {q.j4:7.1f}   "
            f"({expected.x:7.2f},{expected.y:7.2f},{expected.z:7.2f}) "
            f"{error:8.4f} {droop:6.3f}d {tilt:6.3f}d {unexplained:8.4f}"
            f"{'  <-- FAIL' if bad else ''}"
        )
        if error > TCP_TOLERANCE_MM:
            failures.append(f"chain off by {error:.3f}mm at {q}")
        if abs(tilt) > HEAD_TILT_TOLERANCE_DEG:
            failures.append(f"head tilted {tilt:.3f} deg off level at {q}")
        if unexplained > UNEXPLAINED_TOLERANCE_MM:
            failures.append(
                f"{unexplained:.4f}mm of TCP error at {q} is not explained by head "
                f"tilt -- the URDF chain does not match the kinematic model"
            )
    return failures


def check_drives(world: World, arm: SimArm) -> list[str]:
    """Command a pose through the position drives and see where the joints land."""
    failures = []
    print("\n-- position drives settle at the commanded pose ---------------------")
    arm.park()
    step(world, 30)

    target = JointAnglesDeg(30.0, 95.0, -20.0, 45.0)
    arm.set_model_angles(target)
    # A position drive needs time; 2s of sim at 60Hz is plenty for this reach.
    step(world, 120)
    reached = arm.state().q

    for name in ("j1", "j2", "j3", "j4"):
        want, got = getattr(target, name), getattr(reached, name)
        error = abs(want - got)
        flag = "" if error <= DRIVE_TOLERANCE_DEG else "  <-- FAIL"
        print(f"  {name}: commanded {want:8.2f}  reached {got:8.2f}  err {error:6.3f}{flag}")
        if error > DRIVE_TOLERANCE_DEG:
            failures.append(f"{name} settled {error:.2f} deg from its target")
    return failures


def render_and_detect(world: World, arm: SimArm) -> list[str]:
    """Render the scene camera and try to decode the desk's ArUco tags."""
    from isaacsim.sensors.camera import Camera

    failures = []
    OUT.mkdir(parents=True, exist_ok=True)

    arm.park()
    # Park puts the arm over the desk; open jaws so the gripper reads as it does
    # live, where the camera sees the arm between moves.
    arm.set_gripper_s(GRIPPER_S_OPEN)
    step(world, 60)

    camera = Camera(prim_path="/World/SceneCamera", resolution=rig.CAM_RESOLUTION)
    camera.initialize()
    # The RTX renderer needs several frames before the first is fully converged;
    # an unconverged frame is noisy enough to cost tag decodes.
    for _ in range(40):
        world.step(render=True)

    frame = camera.get_rgb()
    if frame is None or frame.size == 0:
        return ["scene camera returned no frame"]

    bgr = cv2.cvtColor(np.asarray(frame, dtype=np.uint8)[..., :3], cv2.COLOR_RGB2BGR)
    path = OUT / "scene_camera.png"
    cv2.imwrite(str(path), bgr)
    print(f"\n-- scene camera ----------------------------------------------------")
    print(f"  wrote {path}  {bgr.shape[1]}x{bgr.shape[0]}")

    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
        cv2.aruco.DetectorParameters(),
    )
    corners, ids, _ = detector.detectMarkers(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY))
    found = sorted(int(v) for v in ids.flatten()) if ids is not None else []
    expected = sorted(m.tag_id for m in rig.MARKERS)
    print(f"  ArUco expected {expected}")
    print(f"  ArUco decoded  {found}")

    detected = {int(i): c.reshape(4, 2) for i, c in zip(ids.flatten(), corners)} if ids is not None else {}
    if corners:
        cv2.aruco.drawDetectedMarkers(bgr, corners, ids)
    cv2.imwrite(str(OUT / "scene_camera_tags.png"), bgr)

    missing = [t for t in expected if t not in found]
    if missing:
        failures.append(f"tags {missing} did not decode from the simulated camera")
    # A tag that decodes as a *different* tag is worse than one that does not
    # decode: it silently moves a calibration point across the desk.
    wrong = sorted({t for t in found if t not in expected})
    if wrong:
        failures.append(f"the simulated camera decoded tags {wrong}, which are not on the desk")

    failures += check_tags_through_calibration(detected)
    # Tag drawing mutated bgr, so detection runs on a clean copy.
    return failures + check_cube_detection(
        cv2.cvtColor(np.asarray(frame, dtype=np.uint8)[..., :3], cv2.COLOR_RGB2BGR)
    )


def check_tags_through_calibration(detected: dict) -> list[str]:
    """Read the simulated frame with the *real* rig's calibration file.

    The tags decoding only proves they are legible. This proves the sim is in
    the same place as the rig it copies: every tag's pixels are pushed through
    ``vision_calibration.json``'s own homography, exactly as the live stack
    would, and compared against where that same file says the tag is taped.

    The residual is not zero and cannot be. The real lens has barrel distortion
    that a pinhole cannot express, so a sim camera pinned at the measured lens
    position reproduces the rig's table map to about 20px -- ``rig.SCENE_CAMERA``
    reports that fit. Anything much past it is a placement error, not optics.
    """
    from mt4_sim.calibration import CALIBRATION

    failures = []
    print("\n-- simulated tags, read through the live vision_calibration.json --")
    print(
        f"  {'tag':>4}  {'calibration says':>22}  {'sim frame reads':>22}  "
        f"{'off':>7}  {'yaw err':>8}  {'side':>7}"
    )
    for placement in rig.MARKERS:
        seen = detected.get(placement.tag_id)
        if seen is None:
            continue
        on_table = np.array([CALIBRATION.pixel_to_robot(float(u), float(v)) for u, v in seen])
        centre = on_table.mean(axis=0)
        edge = on_table[1] - on_table[0]
        yaw = math.degrees(math.atan2(edge[1], edge[0]))
        side = float(
            np.mean([np.linalg.norm(on_table[(i + 1) % 4] - on_table[i]) for i in range(4)])
        )
        off = math.dist(centre, (placement.x_mm, placement.y_mm))
        yaw_error = (yaw - placement.yaw_deg + 180.0) % 360.0 - 180.0
        bad = off > TAG_PLACEMENT_TOLERANCE_MM or abs(yaw_error) > TAG_YAW_TOLERANCE_DEG
        print(
            f"  {placement.tag_id:>4}  ({placement.x_mm:8.1f},{placement.y_mm:8.1f})  "
            f"({centre[0]:8.1f},{centre[1]:8.1f})  {off:6.1f}mm  {yaw_error:7.2f}d  "
            f"{side:6.1f}mm{'  <-- FAIL' if bad else ''}"
        )
        if off > TAG_PLACEMENT_TOLERANCE_MM:
            failures.append(
                f"tag {placement.tag_id} reads {off:.1f}mm from where the calibration "
                f"says it is taped -- past what the pinhole/lens mismatch explains"
            )
        if abs(yaw_error) > TAG_YAW_TOLERANCE_DEG:
            failures.append(
                f"tag {placement.tag_id} is laid at {yaw_error:.1f} deg from the "
                f"orientation the rig's own corners record"
            )
    return failures


def check_cube_detection(bgr) -> list[str]:
    """Run the live HSV thresholds over the rendered frame.

    The tags decoding proves the geometry renders; this proves the *materials*
    do. ``mt4_vision.detect`` thresholds hue and saturation, and a physically
    plausible render under a bright dome light desaturates every coloured face
    enough to fall out of its own band -- so a cube can be plainly visible in the
    image and invisible to the detector.
    """
    from mt4_vision.detect import COLOR_RANGES, MIN_BLOB_AREA, band_mask

    failures = []
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    print("\n-- cubes vs mt4_vision.detect's own HSV bands ----------------------")
    print(f"  a blob must clear MIN_BLOB_AREA = {MIN_BLOB_AREA:.0f} px^2")

    wanted = {c.color for c in rig.CUBES}
    for color in sorted(wanted):
        mask = band_mask(hsv, COLOR_RANGES[color])
        count, _labels, stats, _cent = cv2.connectedComponentsWithStats(mask, connectivity=8)
        blobs = sorted(
            (int(stats[i, cv2.CC_STAT_AREA]) for i in range(1, count)), reverse=True
        )
        biggest = blobs[0] if blobs else 0
        ok = biggest >= MIN_BLOB_AREA
        print(
            f"  {color:7s} largest blob {biggest:6d} px^2  "
            f"(blobs over floor: {sum(1 for b in blobs if b >= MIN_BLOB_AREA)})"
            f"{'' if ok else '  <-- FAIL'}"
        )
        if not ok:
            failures.append(
                f"{color} cube's largest blob is {biggest} px^2, under the detector's "
                f"{MIN_BLOB_AREA:.0f} floor -- the render is too desaturated for "
                f"mt4_vision.detect"
            )
    return failures


def render_preview(world: World) -> None:
    """A third-person view, so the rig can be eyeballed without opening the GUI.

    Kept above the work surface and out past its front corner: the wall sits
    just behind the arm, so anything looking from behind sees the back of it.
    """
    from mt4_sim.scene import define_camera
    from mt4_sim.chain import DESK_Z_MM as desk

    define_camera(
        world.stage,
        "/World/PreviewCamera",
        eye_mm=(610.0, -470.0, desk + 340.0),
        target_mm=(130.0, -10.0, desk + 10.0),
        resolution=(1600, 1000),
        fov_deg=55.0,
    )

    from isaacsim.sensors.camera import Camera

    camera = Camera(prim_path="/World/PreviewCamera", resolution=(1600, 1000))
    camera.initialize()
    for _ in range(40):
        world.step(render=True)
    frame = camera.get_rgb()
    if frame is None or frame.size == 0:
        print("  preview camera returned no frame")
        return
    bgr = cv2.cvtColor(np.asarray(frame, dtype=np.uint8)[..., :3], cv2.COLOR_RGB2BGR)
    path = OUT / "preview.png"
    cv2.imwrite(str(path), bgr)
    print(f"  wrote {path}")


def report_camera_geometry() -> None:
    """Where the sim's lens is, and how well it stands in for the rig's."""
    from mt4_sim.calibration import CALIB_PATH

    cam = rig.SCENE_CAMERA
    eye = cam.position_mm
    print("\n-- scene camera vs the rig's own calibration -----------------------")
    print(f"  read from {CALIB_PATH}")
    print(
        f"  lens at nadir ({eye[0]:6.1f}, {eye[1]:6.1f}), "
        f"{eye[2] - rig.DESK_TOP_Z_MM:5.1f}mm above the table -- as measured"
    )
    print(
        f"  aimed at ({cam.target_mm[0]:6.1f}, {cam.target_mm[1]:6.1f}), "
        f"{cam.horizontal_fov_deg:5.2f} deg across {cam.resolution[0]}x{cam.resolution[1]}"
    )
    print(
        f"  reproduces the calibration's pixel<->table map to "
        f"{cam.residual_px:.1f} px rms ({cam.residual_mm:.1f} mm), which is the "
        f"real lens's distortion"
    )


def main() -> int:
    if not SCENE_USD.is_file():
        print(f"missing {SCENE_USD} -- run scripts/build_scene.py first")
        return 1

    open_stage(str(SCENE_USD))
    world = World(stage_units_in_meters=1.0, physics_dt=physics_dt())
    world.reset()

    arm = SimArm()
    print(f"articulation DOFs: {list(arm._art.dof_names)}")

    failures: list[str] = []
    failures += check_chain(world, arm)
    failures += check_drives(world, arm)
    failures += render_and_detect(world, arm)
    render_preview(world)
    report_camera_geometry()

    print("\n====================================================================")
    if failures:
        print(f"{len(failures)} check(s) FAILED:")
        for f in failures:
            print(f"  - {f}")
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    code = main()
    app.close()
    sys.exit(code)
