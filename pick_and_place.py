"""Simple pick-and-place for the Viam Python SDK.

Moves the arm through a lift-before-shift pick-and-place with direct arm
commands (no motion planner):

    START (above pick) -> PICK -> grab -> lift back up ->
    above place -> PLACE -> release -> retreat up

Waypoints are derived from the pick pose: START/lift is START_HEIGHT above
PICK, PLACE is PLACE_LEFT_OFFSET to the side, and the place approach is
START_HEIGHT above PLACE. Lifting the object before moving sideways keeps it
clear of the surface, and approaching each pose from directly above keeps the
vertical segments roughly straight down.

The core motion lives in `execute_pick_place(arm, gripper, pick_pose)` so other
scripts (e.g. detect_and_pick.py) can reuse it with a different pick pose.

Credentials and resource names come from environment variables (see
`.env.example`). `connect()` lives here too, and is shared with
detect_and_pick.py: set VIAM_LOCAL_ADDRESS to dial the machine directly rather
than relaying through the cloud, which is what you want when running on the
device that hosts viam-server.

Run:

    python pick_and_place.py
"""

import asyncio
import os

from viam.components.arm import Arm
from viam.components.gripper import Gripper
from viam.logging import getLogger
from viam.proto.common import Pose
from viam.robot.client import RobotClient
from viam.rpc.dial import DialOptions

LOGGER = getLogger(__name__)

# ---------------------------------------------------------------------------
# Credentials (from your machine's CONNECT tab in the Viam app)
# ---------------------------------------------------------------------------
API_KEY = os.environ.get("VIAM_API_KEY", "")
API_KEY_ID = os.environ.get("VIAM_API_KEY_ID", "")

# Cloud address of the machine. Traffic is relayed through Viam's servers over
# WebRTC: convenient from a laptop, but large payloads get split across many
# data-channel messages.
MACHINE_ADDRESS = os.environ.get("VIAM_MACHINE_ADDRESS", "")

# Set this to dial the machine directly over gRPC instead of relaying through
# the cloud. Running on the same device as viam-server, that is its default
# bind address, "localhost:8080"; from elsewhere on the LAN, use the machine's
# local address from the CONNECT tab. Include the port either way, or the SDK
# assumes 443.
#
# This matters most for perception: a 640x480 depth frame is ~600 KB, big
# enough that WebRTC chunking makes it arrive late or incomplete under load.
# A direct connection delivers the frame in one piece.
LOCAL_ADDRESS = os.environ.get("VIAM_LOCAL_ADDRESS", "")

# viam-server normally serves TLS even locally, and the SDK skips certificate
# verification for localhost, so the default works as-is. Set this only if a
# local connection fails with InsecureConnectionError, which means your server
# is serving plain HTTP/2; it lets the SDK fall back to an unencrypted
# connection, which also sends the API key in the clear.
LOCAL_ALLOW_INSECURE = os.environ.get("VIAM_LOCAL_INSECURE", "0") == "1"

# Resource names as configured on your machine.
ARM_NAME = os.environ.get("VIAM_ARM_NAME", "arm-1")
GRIPPER_NAME = os.environ.get("VIAM_GRIPPER_NAME", "gripper-1")

# ---------------------------------------------------------------------------
# Poses  (positions in millimeters, in the arm's base frame)
# ---------------------------------------------------------------------------
# Orientation captured at the grasp point. The gripper points nearly straight
# down; note this is ~10 deg off true vertical (o_x = -0.17), which is your
# real measured orientation. For mathematically straight down, instead use
# {"o_x": 0.0, "o_y": 0.0, "o_z": -1.0, "theta": 0.0}.
GRASP = {"o_x": -0.17446, "o_y": -0.00274, "o_z": -0.98466, "theta": -179.8715}

# START sits this far directly above PICK; PLACE is offset sideways (mm).
START_HEIGHT = 200.0
PLACE_LEFT_OFFSET = 25.0  # +y = left in the arm base frame; flip to -25.0 if
# the arm moves the wrong direction

# Pauses (seconds) around the grab so the object is not disturbed mid-grasp:
# SETTLE lets the arm come fully to rest before the gripper closes; CLAMP lets
# the gripper finish clamping before the arm lifts. Increase if it still slips.
SETTLE_SECONDS = 1.0
CLAMP_SECONDS = 1.5

# PICK is your measured grasp point (z 214.7491), lowered 2 mm so the gripper
# reaches down far enough to grab the object.
PICK_POSE = Pose(x=472.0814, y=7.3932, z=212.7491, **GRASP)


def machine_address() -> str:
    """The address to dial: the local one if set, otherwise the cloud one."""
    return LOCAL_ADDRESS or MACHINE_ADDRESS


def connect_options() -> "RobotClient.Options":
    """Build connection options, failing early if credentials are missing."""
    missing = [
        name
        for name, value in (("VIAM_API_KEY", API_KEY), ("VIAM_API_KEY_ID", API_KEY_ID))
        if not value
    ]
    if not machine_address():
        missing.append("VIAM_MACHINE_ADDRESS (or VIAM_LOCAL_ADDRESS)")
    if missing:
        raise SystemExit(
            "Missing required environment variables: "
            + ", ".join(missing)
            + "\nSee .env.example and set them before running."
        )

    dial_options = DialOptions.with_api_key(api_key=API_KEY, api_key_id=API_KEY_ID)
    if LOCAL_ADDRESS:
        # Dial the machine's gRPC endpoint directly: no signaling, no relay,
        # and no chunking of large frames.
        dial_options.disable_webrtc = True
        # TLS is still attempted first; this only permits the fallback.
        dial_options.allow_insecure_with_creds_downgrade = LOCAL_ALLOW_INSECURE
    return RobotClient.Options(dial_options=dial_options)


async def connect() -> RobotClient:
    """Connect to the machine, directly if VIAM_LOCAL_ADDRESS is set."""
    options = connect_options()
    address = machine_address()
    LOGGER.info(
        "Connecting to %s (%s)",
        address,
        "direct gRPC" if LOCAL_ADDRESS else "cloud relay over WebRTC",
    )
    return await RobotClient.at_address(address, options)


def offset_pose(pose: Pose, *, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0) -> Pose:
    """Return a copy of `pose` shifted by (dx, dy, dz) mm, keeping orientation."""
    return Pose(
        x=pose.x + dx,
        y=pose.y + dy,
        z=pose.z + dz,
        o_x=pose.o_x,
        o_y=pose.o_y,
        o_z=pose.o_z,
        theta=pose.theta,
    )


async def move_to(arm: Arm, pose: Pose, label: str) -> None:
    """Move the arm's end effector to `pose`."""
    LOGGER.info("Moving to %s: x=%.1f y=%.1f z=%.1f", label, pose.x, pose.y, pose.z)
    await arm.move_to_position(pose)


async def execute_pick_place(arm: Arm, gripper: Gripper, pick_pose: Pose) -> None:
    """Run one lift-before-shift pick-and-place cycle, picking at `pick_pose`.

    Derived waypoints (orientation is inherited from `pick_pose`):
        start / lift   = pick_pose raised by START_HEIGHT
        place          = pick_pose shifted by PLACE_LEFT_OFFSET in y
        place_approach = place raised by START_HEIGHT
    """
    start = offset_pose(pick_pose, dz=START_HEIGHT)
    place = offset_pose(pick_pose, dy=PLACE_LEFT_OFFSET)
    place_approach = offset_pose(place, dz=START_HEIGHT)

    LOGGER.info("Opening gripper")
    await gripper.open()

    # Descend onto the object and grab it.
    await move_to(arm, start, "START (above pick)")
    await move_to(arm, pick_pose, "PICK")

    # Let the arm come fully to rest before closing, so it is not still moving
    # when the gripper tries to grasp the object.
    LOGGER.info("Settling %.1fs before grabbing", SETTLE_SECONDS)
    await asyncio.sleep(SETTLE_SECONDS)

    LOGGER.info("Closing gripper to grab the object")
    grabbed = await gripper.grab()
    if not grabbed:
        LOGGER.warning("Gripper did not report a successful grab")

    # Hold at the pick location to let the gripper finish clamping before lifting.
    LOGGER.info("Holding %.1fs for the gripper to clamp", CLAMP_SECONDS)
    await asyncio.sleep(CLAMP_SECONDS)

    # Lift straight up so the object clears the surface before moving sideways.
    await move_to(arm, start, "LIFT (above pick)")

    # Travel over the place location at height, then lower onto it.
    await move_to(arm, place_approach, "APPROACH (above place)")
    await move_to(arm, place, "PLACE")

    LOGGER.info("Opening gripper to release the object")
    await gripper.open()

    # Retreat up so the gripper clears the placed object.
    await move_to(arm, place_approach, "RETREAT (above place)")

    LOGGER.info("Pick-and-place complete")


async def main() -> None:
    machine = await connect()
    try:
        LOGGER.info("Connected. Available resources: %s", machine.resource_names)
        arm = Arm.from_robot(machine, ARM_NAME)
        gripper = Gripper.from_robot(machine, GRIPPER_NAME)
        await execute_pick_place(arm, gripper, PICK_POSE)
    finally:
        await machine.close()


if __name__ == "__main__":
    asyncio.run(main())
