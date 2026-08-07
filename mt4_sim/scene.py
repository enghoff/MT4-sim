"""Build the MT4 work-surface scene on a USD stage.

Everything is authored in the arm's home-angle frame converted to metres, so a
robot-frame coordinate from the real stack addresses the same point here: a cube
the live scene reports at (230, -60) sits at (0.230, -0.060) in the stage.

The stage is not self-contained by design -- ``/World/MT4`` references the
articulation the URDF importer produced, so re-importing the arm updates the
scene without rebuilding it.
"""

from __future__ import annotations

import math
from pathlib import Path

from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade

from mt4_sim import rig
from mt4_sim.chain import DESK_Z_MM, MM, park_pose, urdf_from_model
from mt4_sim.markers import quiet_zone_fraction, write_tag_textures

WORLD = "/World"
ARM_PATH = f"{WORLD}/MT4"


def _rgb(v: tuple[float, float, float]) -> Gf.Vec3f:
    return Gf.Vec3f(*v)


def _preview_material(stage, path: str, rgb: tuple[float, float, float], rough: float = 0.6):
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, f"{path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(_rgb(rgb))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def _textured_material(stage, path: str, texture: Path):
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, f"{path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.85)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)

    reader = UsdShade.Shader.Define(stage, f"{path}/UVReader")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)

    tex = UsdShade.Shader.Define(stage, f"{path}/Texture")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(texture.as_posix())
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        reader.ConnectableAPI(), "result"
    )
    # Repeating a tag would let the detector see partial copies at the seam.
    tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("clamp")
    tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
    tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)

    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        tex.ConnectableAPI(), "rgb"
    )
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def _bind(prim, material) -> None:
    UsdShade.MaterialBindingAPI(prim).Bind(material)


def _box(stage, path: str, size_m: tuple[float, float, float], centre_m: tuple[float, float, float]):
    """An axis-aligned box as a scaled UsdGeom.Cube (unit cube is 2m across)."""
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(2.0)
    xform = UsdGeom.Xformable(cube)
    xform.AddTranslateOp().Set(Gf.Vec3d(*centre_m))
    xform.AddScaleOp().Set(Gf.Vec3f(size_m[0] / 2.0, size_m[1] / 2.0, size_m[2] / 2.0))
    return cube


def _quad(stage, path: str, side_m: float):
    """A unit-UV quad in the XY plane, centred on its origin."""
    mesh = UsdGeom.Mesh.Define(stage, path)
    h = side_m / 2.0
    mesh.CreatePointsAttr(
        [Gf.Vec3f(-h, -h, 0.0), Gf.Vec3f(h, -h, 0.0), Gf.Vec3f(h, h, 0.0), Gf.Vec3f(-h, h, 0.0)]
    )
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    uvs = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying
    )
    uvs.Set([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)])
    return mesh


# --------------------------------------------------------------------------
# Scene pieces
# --------------------------------------------------------------------------


def add_physics_scene(stage) -> None:
    scene = UsdPhysics.Scene.Define(stage, f"{WORLD}/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr(9.81)
    physx = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    physx.CreateEnableCCDAttr(True)
    # 20mm cubes and 8mm-thick fingers are small next to a 1/60s step; 240Hz
    # keeps a closing jaw from tunnelling through a cube between substeps.
    physx.CreateTimeStepsPerSecondAttr(240)


def add_lighting(stage) -> None:
    # Kept dim on purpose. The cube detector thresholds HSV saturation, and a
    # bright dome washes saturation out of every coloured face -- a red cube
    # under a strong dome reads pale enough to fall out of its own hue band.
    dome = UsdLux.DomeLight.Define(stage, f"{WORLD}/Lights/Dome")
    dome.CreateIntensityAttr(180.0)
    dome.CreateColorAttr(Gf.Vec3f(1.0, 0.99, 0.96))

    # A single hard key light would sink one side of every cube below the HSV
    # value floors the detector uses; this pair keeps side faces lit.
    for name, angle_deg, intensity in (("Key", 35.0, 2600.0), ("Fill", -140.0, 1100.0)):
        light = UsdLux.DistantLight.Define(stage, f"{WORLD}/Lights/{name}")
        light.CreateIntensityAttr(intensity)
        light.CreateAngleAttr(1.5)
        xform = UsdGeom.Xformable(light)
        xform.AddRotateXYZOp().Set(Gf.Vec3f(-55.0, 0.0, angle_deg))


def add_desk(stage) -> None:
    x0, x1 = (v * MM for v in rig.DESK_X_MM)
    y0, y1 = (v * MM for v in rig.DESK_Y_MM)
    thickness = rig.DESK_THICKNESS_MM * MM
    top = DESK_Z_MM * MM

    desk = _box(
        stage,
        f"{WORLD}/Desk",
        (x1 - x0, y1 - y0, thickness),
        ((x0 + x1) / 2.0, (y0 + y1) / 2.0, top - thickness / 2.0),
    )
    UsdPhysics.CollisionAPI.Apply(desk.GetPrim())
    _bind(desk.GetPrim(), _preview_material(stage, f"{WORLD}/Looks/Desk", rig.DESK_RGB, 0.8))

    height = rig.WALL_HEIGHT_MM * MM
    wall = _box(
        stage,
        f"{WORLD}/Wall",
        (0.01, y1 - y0, height),
        (rig.WALL_X_MM * MM, (y0 + y1) / 2.0, top + height / 2.0),
    )
    UsdPhysics.CollisionAPI.Apply(wall.GetPrim())
    _bind(wall.GetPrim(), _preview_material(stage, f"{WORLD}/Looks/Wall", rig.WALL_RGB, 0.9))


def add_markers(stage, texture_dir: Path) -> None:
    textures = write_tag_textures(texture_dir)
    # The card carries a quiet zone the printed tag does not, so scale the card
    # up to keep the black tag itself at MARKER_SIZE_MM.
    card_side = rig.MARKER_SIZE_MM * quiet_zone_fraction() * MM
    UsdGeom.Scope.Define(stage, f"{WORLD}/Markers")

    for marker in rig.MARKERS:
        path = f"{WORLD}/Markers/marker_{marker.tag_id}"
        mesh = _quad(stage, path, card_side)
        xform = UsdGeom.Xformable(mesh)
        # Tags are taped down: a hair above the desk so they never z-fight.
        xform.AddTranslateOp().Set(
            Gf.Vec3d(marker.x_mm * MM, marker.y_mm * MM, DESK_Z_MM * MM + 0.0002)
        )
        xform.AddRotateZOp().Set(marker.yaw_deg)
        _bind(
            mesh.GetPrim(),
            _textured_material(
                stage, f"{WORLD}/Looks/Marker_{marker.tag_id}", textures[marker.tag_id]
            ),
        )


def add_cubes(stage) -> None:
    side = rig.CUBE_SIZE_MM * MM
    UsdGeom.Scope.Define(stage, f"{WORLD}/Cubes")

    for index, cube in enumerate(rig.CUBES, start=1):
        path = f"{WORLD}/Cubes/cube_{index}_{cube.color}"
        prim_cube = UsdGeom.Cube.Define(stage, path)
        prim_cube.CreateSizeAttr(2.0)
        xform = UsdGeom.Xformable(prim_cube)
        xform.AddTranslateOp().Set(
            Gf.Vec3d(cube.x_mm * MM, cube.y_mm * MM, DESK_Z_MM * MM + side / 2.0)
        )
        xform.AddRotateZOp().Set(cube.yaw_deg)
        xform.AddScaleOp().Set(Gf.Vec3f(side / 2.0, side / 2.0, side / 2.0))

        prim = prim_cube.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        rigid = UsdPhysics.RigidBodyAPI.Apply(prim)
        rigid.CreateRigidBodyEnabledAttr(True)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(0.008)
        # A grasp holds by friction alone -- there is no suction on this arm --
        # so the jaw/cube pair needs a grippy contact or picks slide out.
        material = UsdPhysics.MaterialAPI.Apply(prim)
        material.CreateStaticFrictionAttr(1.1)
        material.CreateDynamicFrictionAttr(1.0)
        material.CreateRestitutionAttr(0.0)
        _bind(prim, _preview_material(stage, f"{WORLD}/Looks/Cube_{index}", rig.CUBE_RGB[cube.color]))


def look_at(eye_m: Gf.Vec3d, target_m: Gf.Vec3d) -> Gf.Matrix4d:
    """Camera-to-world transform for a lens at ``eye_m`` aimed at ``target_m``.

    A USD camera looks down its own -Z, so this is the inverse of a view matrix.
    The reference up is world +Z unless the view is within a degree of vertical,
    where +Z gives no unique roll and +Y is used instead.
    """
    forward = (target_m - eye_m).GetNormalized()
    up = Gf.Vec3d(0, 0, 1)
    if abs(Gf.Dot(forward, up)) > 0.9998:
        up = Gf.Vec3d(0, 1, 0)
    return Gf.Matrix4d().SetLookAt(eye_m, target_m, up).GetInverse()


def define_camera(stage, path: str, eye_mm, target_mm, resolution, fov_deg: float):
    camera = UsdGeom.Camera.Define(stage, path)
    width, height = resolution
    aperture = 24.0  # mm of sensor; focal length follows from the field of view
    camera.CreateHorizontalApertureAttr(aperture)
    camera.CreateVerticalApertureAttr(aperture * height / width)
    camera.CreateFocalLengthAttr(aperture / (2.0 * math.tan(math.radians(fov_deg) / 2.0)))
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.01, 20.0))

    eye = Gf.Vec3d(*(v * MM for v in eye_mm))
    target = Gf.Vec3d(*(v * MM for v in target_mm))
    UsdGeom.Xformable(camera).AddTransformOp().Set(look_at(eye, target))
    return camera


def add_scene_camera(stage) -> None:
    """The oblique scene camera: the sim's stand-in for the rig's USB webcam."""
    define_camera(
        stage,
        f"{WORLD}/SceneCamera",
        rig.CAM_POSITION_MM,
        rig.CAM_TARGET_MM,
        rig.CAM_RESOLUTION,
        rig.CAM_HORIZONTAL_FOV_DEG,
    )


def add_arm(stage, arm_usd: Path) -> None:
    prim = stage.DefinePrim(ARM_PATH, "Xform")
    prim.GetReferences().AddReference(arm_usd.as_posix())


def set_park_pose(stage) -> list[str]:
    """Open the stage in the pose the arm holds after homing.

    A stage with every joint at zero folds the arm through itself -- q2 = 0
    lays the upper arm flat below its own soft limit of 22.7 deg. Both the drive
    target and the joint's start state are written, so the arm is already parked
    before the first physics step rather than swinging there from the fold.

    Returns the joint paths it wrote, for the caller to report.
    """
    from mt4_sim.chain import ARM_JOINT_NAMES, FINGER_JOINT_NAMES

    angles_deg = {
        name: math.degrees(value)
        for name, value in zip(ARM_JOINT_NAMES, urdf_from_model(park_pose()))
    }
    # Closed jaws: the real arm parks closed, and open jaws in the start state
    # would drive shut on the first step.
    fingers_m = dict.fromkeys(FINGER_JOINT_NAMES, 0.0)

    written: list[str] = []
    for name, value, kind in (
        *((n, v, "angular") for n, v in angles_deg.items()),
        *((n, v, "linear") for n, v in fingers_m.items()),
    ):
        prim = stage.GetPrimAtPath(f"{ARM_PATH}/Physics/{name}")
        if not prim.IsValid():
            raise RuntimeError(f"no joint prim at {ARM_PATH}/Physics/{name}")

        drive = UsdPhysics.DriveAPI.Get(prim, kind)
        if not drive:
            raise RuntimeError(f"{name} has no {kind} drive to target")
        drive.CreateTargetPositionAttr(value)

        # PhysicsJointStateAPI is already applied by the importer; only the
        # value is missing. Setting it directly avoids needing the Physx schema
        # bindings here.
        state = prim.GetAttribute(f"state:{kind}:physics:position")
        if not state.IsValid():
            raise RuntimeError(f"{name} has no state:{kind}:physics:position attribute")
        state.Set(value)
        written.append(f"{name} = {value:.4f} ({kind})")

    return written


def build(stage, arm_usd: Path, texture_dir: Path) -> list[str]:
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, WORLD)
    stage.SetDefaultPrim(world.GetPrim())

    add_physics_scene(stage)
    add_lighting(stage)
    add_desk(stage)
    add_markers(stage, texture_dir)
    add_cubes(stage)
    add_scene_camera(stage)
    add_arm(stage, arm_usd)
    return set_park_pose(stage)
