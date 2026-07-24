"""Detection-driven pick-and-place for the Viam Python SDK.

Instead of a fixed pick pose, this detects a target object with the vision
service, computes its 3D position from the RGBD camera, transforms it into the
arm's frame, and runs the pick-and-place sequence to that position.

Pipeline:
    1. vision service -> 2D detections (boxes + class names)
    2. select the highest-confidence detection whose class == TARGET_CLASS
       (TARGET_CLASS is your "describe which object")
    3. box-center pixel + depth (from the RGBD camera) -> 3D point in the
       camera frame, deprojected with the camera's own intrinsics
    4. transform_pose(camera frame -> world) via Viam's frame system
    5. pick at that (x, y) with your measured grasp orientation, then place

SAFETY: motion is OFF by default. The script prints the detection and the
computed pick pose so you can validate the whole chain first. Only once the
poses look right AND your detector is trained on your objects, enable motion
with VIAM_EXECUTE=1.

Reuses connection, grasp orientation, dwell timing, and the motion sequence
from pick_and_place.py.

Run (detect-only):        python detect_and_pick.py
Run (actually move):      VIAM_EXECUTE=1 python detect_and_pick.py
Pick a specific class:    VIAM_TARGET_CLASS=gear python detect_and_pick.py
"""

import asyncio
import os
import statistics

from viam.components.arm import Arm
from viam.components.camera import Camera
from viam.components.gripper import Gripper
from viam.logging import getLogger
from viam.proto.common import Pose, PoseInFrame
from viam.robot.client import RobotClient
from viam.services.vision import VisionClient

from pick_and_place import (
    ARM_NAME,
    GRASP,
    GRIPPER_NAME,
    MACHINE_ADDRESS,
    PICK_POSE,
    connect_options,
    execute_pick_place,
)

LOGGER = getLogger(__name__)

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
CAMERA_NAME = os.environ.get("VIAM_CAMERA_NAME", "cam-1")
VISION_NAME = os.environ.get("VIAM_VISION_NAME", "vision-1")

# Frame that the deprojected point is expressed in (the camera's frame in the
# frame system) and the frame we want the pick point in. transform_pose uses
# the machine's configured frame system to convert between them.
CAMERA_FRAME = os.environ.get("VIAM_CAMERA_FRAME", CAMERA_NAME)
DEST_FRAME = os.environ.get("VIAM_DEST_FRAME", "world")

# "Describe which object": the detector class label to pick. If empty, the
# highest-confidence detection of ANY class is used (and its class is logged) -
# handy while the detector is still being trained.
TARGET_CLASS = os.environ.get("VIAM_TARGET_CLASS", "")
MIN_CONFIDENCE = float(os.environ.get("VIAM_MIN_CONFIDENCE", "0.5"))

# SAFETY GATE: motion is disabled unless VIAM_EXECUTE=1. Detect-only otherwise.
EXECUTE_MOTION = os.environ.get("VIAM_EXECUTE", "0") == "1"

# The objects sit on a flat surface at a known, already-tuned grasp height, and
# depth-derived z is noisier than that. So by default we take x/y from the
# detection but keep the proven pick z from pick_and_place.PICK_POSE. Set
# VIAM_USE_DEPTH_Z=1 to grasp at the depth-derived height instead.
USE_DEPTH_Z = os.environ.get("VIAM_USE_DEPTH_Z", "0") == "1"

# Sample depth over a small window and take the median (ignoring zero/hole
# pixels) so one bad pixel does not throw off the position.
DEPTH_WINDOW_HALF = 3

# The remote link occasionally drops large payloads (depth frames); retry.
RETRIES = 3
RETRY_DELAY_S = 1.0


async def with_retries(make_coro, what: str):
    """Await make_coro(), retrying on transient failures (e.g. dropped frames)."""
    last = None
    for attempt in range(1, RETRIES + 1):
        try:
            return await make_coro()
        except Exception as exc:  # noqa: BLE001 - surface after retries
            last = exc
            LOGGER.warning(
                "%s failed (attempt %d/%d): %s", what, attempt, RETRIES, type(exc).__name__
            )
            if attempt < RETRIES:
                await asyncio.sleep(RETRY_DELAY_S)
    raise last


def select_detection(detections, target_class: str, min_conf: float):
    """Return the highest-confidence detection matching the target, or None."""
    candidates = [d for d in detections if d.confidence >= min_conf]
    if target_class:
        candidates = [d for d in candidates if d.class_name == target_class]
    if not candidates:
        return None
    return max(candidates, key=lambda d: d.confidence)


def deproject(u: float, v: float, depth_mm: float, intr) -> tuple:
    """Pixel (u, v) + depth (mm) -> 3D point (mm) in the camera optical frame.

    Optical convention: +x right, +y down, +z forward (out of the lens).
    """
    x = (u - intr.center_x_px) * depth_mm / intr.focal_x_px
    y = (v - intr.center_y_px) * depth_mm / intr.focal_y_px
    z = depth_mm
    return x, y, z


def sample_depth(depth_arr, u: int, v: int, half: int = DEPTH_WINDOW_HALF):
    """Median depth (mm) over a window around (u, v), ignoring zero/hole pixels."""
    height = len(depth_arr)
    width = len(depth_arr[0]) if height else 0
    values = []
    for dv in range(-half, half + 1):
        for du in range(-half, half + 1):
            uu, vv = u + du, v + dv
            if 0 <= vv < height and 0 <= uu < width:
                d = depth_arr[vv][uu]
                if d and d > 0:
                    values.append(d)
    return statistics.median(values) if values else None


async def compute_pick_pose(machine, camera: Camera, vision: VisionClient):
    """Detect the target and return (pick_pose, detection), or raise on failure."""
    detections = await with_retries(
        lambda: vision.get_detections_from_camera(CAMERA_NAME, timeout=25),
        "get_detections_from_camera",
    )
    LOGGER.info("Detector returned %d detection(s)", len(detections))
    det = select_detection(detections, TARGET_CLASS, MIN_CONFIDENCE)
    if det is None:
        want = f"class '{TARGET_CLASS}'" if TARGET_CLASS else "any class"
        raise RuntimeError(
            f"No detection matching {want} at confidence >= {MIN_CONFIDENCE}"
        )

    u = int(round((det.x_min + det.x_max) / 2))
    v = int(round((det.y_min + det.y_max) / 2))
    LOGGER.info(
        "Target: class='%s' conf=%.2f box=(%d,%d)-(%d,%d) center=(%d,%d)",
        det.class_name, det.confidence, det.x_min, det.y_min, det.x_max, det.y_max, u, v,
    )

    props = await with_retries(lambda: camera.get_properties(), "get_properties")
    intr = props.intrinsic_parameters

    images, _ = await with_retries(lambda: camera.get_images(timeout=25), "get_images")
    depth_img = next((i for i in images if i.name == "depth"), None)
    if depth_img is None:
        raise RuntimeError("Camera returned no depth image; is it an RGBD camera?")
    depth_arr = depth_img.bytes_to_depth_array()

    z_mm = sample_depth(depth_arr, u, v)
    if z_mm is None:
        raise RuntimeError(
            f"No valid depth near pixel ({u},{v}); object may be out of range"
        )
    LOGGER.info("Depth at target center: %.1f mm", z_mm)

    cam_x, cam_y, cam_z = deproject(u, v, z_mm, intr)
    LOGGER.info("Point in camera frame: x=%.1f y=%.1f z=%.1f mm", cam_x, cam_y, cam_z)

    # Transform the point from the camera frame into the destination frame using
    # the machine's frame system. Orientation here is a placeholder; we only use
    # the returned position and pair it with the measured GRASP orientation.
    cam_pif = PoseInFrame(
        reference_frame=CAMERA_FRAME,
        pose=Pose(x=cam_x, y=cam_y, z=cam_z, o_x=0.0, o_y=0.0, o_z=-1.0, theta=0.0),
    )
    dest_pif = await with_retries(
        lambda: machine.transform_pose(cam_pif, DEST_FRAME), "transform_pose"
    )
    p = dest_pif.pose
    LOGGER.info("Object in '%s' frame: x=%.1f y=%.1f z=%.1f", DEST_FRAME, p.x, p.y, p.z)

    pick_z = p.z if USE_DEPTH_Z else PICK_POSE.z
    if not USE_DEPTH_Z:
        LOGGER.info("Using detected x/y with fixed grasp z=%.4f (set VIAM_USE_DEPTH_Z=1 to use depth z)", pick_z)

    pick_pose = Pose(x=p.x, y=p.y, z=pick_z, **GRASP)
    return pick_pose, det


async def main() -> None:
    machine = await RobotClient.at_address(MACHINE_ADDRESS, connect_options())
    try:
        LOGGER.info("Connected. Target class: %r (min conf %.2f)", TARGET_CLASS or "<any>", MIN_CONFIDENCE)
        camera = Camera.from_robot(machine, CAMERA_NAME)
        vision = VisionClient.from_robot(machine, VISION_NAME)

        try:
            pick_pose, det = await compute_pick_pose(machine, camera, vision)
        except RuntimeError as exc:
            # Expected conditions (no matching detection, no valid depth): report
            # cleanly instead of dumping a traceback.
            LOGGER.warning("No pick this cycle: %s", exc)
            return

        LOGGER.info(
            "==> PICK POSE: x=%.2f y=%.2f z=%.2f  (from '%s' conf=%.2f)",
            pick_pose.x, pick_pose.y, pick_pose.z, det.class_name, det.confidence,
        )

        if not EXECUTE_MOTION:
            LOGGER.info(
                "DETECT-ONLY (no motion). Validate the pick pose above, then rerun "
                "with VIAM_EXECUTE=1 to move the arm."
            )
            return

        arm = Arm.from_robot(machine, ARM_NAME)
        gripper = Gripper.from_robot(machine, GRIPPER_NAME)
        await execute_pick_place(arm, gripper, pick_pose)
    finally:
        await machine.close()


if __name__ == "__main__":
    asyncio.run(main())
