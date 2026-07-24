"""Simple pick-and-place for the Viam Python SDK.

Moves the arm through a lift-before-shift pick-and-place with direct arm
commands (no motion planner):

    START (above pick) -> PICK -> grab -> lift back up ->
    above place -> PLACE -> release -> retreat up

    START_POSE      gripper open, positioned directly above the object
    PICK_POSE       gripper closes here to grab the object
    PLACE_POSE      where the object is released
    PLACE_APPROACH  directly above PLACE, so the arm lowers/lifts vertically

Each step is a direct `move_to_position`. Lifting the object before moving
sideways keeps it clear of the surface, and approaching each pose from
directly above keeps the vertical segments roughly straight down.

Credentials and resource names come from environment variables (see
`.env.example`). START and PLACE are derived from the measured PICK pose
below. Run:

    python pick_and_place.py
"""

import asyncio
import os

from viam.components.arm import Arm
from viam.components.gripper import Gripper
from viam.logging import getLogger
from viam.proto.common import Pose
from viam.robot.client import RobotClient

LOGGER = getLogger(__name__)

# ---------------------------------------------------------------------------
# Credentials (from your machine's CONNECT tab in the Viam app)
# ---------------------------------------------------------------------------
API_KEY = os.environ.get("VIAM_API_KEY", "")
API_KEY_ID = os.environ.get("VIAM_API_KEY_ID", "")
MACHINE_ADDRESS = os.environ.get("VIAM_MACHINE_ADDRESS", "")

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
# reaches down far enough to grab the object. Everything else is derived from
# PICK, so START and PLACE follow this height automatically.
PICK_POSE = Pose(x=472.0814, y=7.3932, z=212.7491, **GRASP)
# Waypoint directly above PICK: both the start position and the lift target.
START_POSE = Pose(x=PICK_POSE.x, y=PICK_POSE.y, z=PICK_POSE.z + START_HEIGHT, **GRASP)
# Drop location, 25 mm to the left of PICK at the same height.
PLACE_POSE = Pose(x=PICK_POSE.x, y=PICK_POSE.y + PLACE_LEFT_OFFSET, z=PICK_POSE.z, **GRASP)
# Waypoint directly above PLACE: approached before lowering, retreated to after.
PLACE_APPROACH = Pose(x=PLACE_POSE.x, y=PLACE_POSE.y, z=PLACE_POSE.z + START_HEIGHT, **GRASP)


def connect_options() -> "RobotClient.Options":
    """Build connection options, failing early if credentials are missing."""
    missing = [
        name
        for name, value in (
            ("VIAM_API_KEY", API_KEY),
            ("VIAM_API_KEY_ID", API_KEY_ID),
            ("VIAM_MACHINE_ADDRESS", MACHINE_ADDRESS),
        )
        if not value
    ]
    if missing:
        raise SystemExit(
            "Missing required environment variables: "
            + ", ".join(missing)
            + "\nSee .env.example and set them before running."
        )
    return RobotClient.Options.with_api_key(api_key=API_KEY, api_key_id=API_KEY_ID)


async def move_to(arm: Arm, pose: Pose, label: str) -> None:
    """Move the arm's end effector to `pose`."""
    LOGGER.info("Moving to %s: x=%.1f y=%.1f z=%.1f", label, pose.x, pose.y, pose.z)
    await arm.move_to_position(pose)


async def pick_and_place(arm: Arm, gripper: Gripper) -> None:
    """Run one pick-and-place cycle with a lift between pick and place.

    START (above pick) -> PICK -> grab -> lift back to START ->
    PLACE_APPROACH (above place) -> PLACE -> release -> retreat.
    Lifting before moving sideways keeps the object clear of the surface.
    """
    LOGGER.info("Opening gripper")
    await gripper.open()

    # Descend onto the object and grab it.
    await move_to(arm, START_POSE, "START (above pick)")
    await move_to(arm, PICK_POSE, "PICK")

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
    await move_to(arm, START_POSE, "LIFT (above pick)")

    # Travel over the place location at height, then lower onto it.
    await move_to(arm, PLACE_APPROACH, "APPROACH (above place)")
    await move_to(arm, PLACE_POSE, "PLACE")

    LOGGER.info("Opening gripper to release the object")
    await gripper.open()

    # Retreat up so the gripper clears the placed object.
    await move_to(arm, PLACE_APPROACH, "RETREAT (above place)")

    LOGGER.info("Pick-and-place complete")


async def main() -> None:
    machine = await RobotClient.at_address(MACHINE_ADDRESS, connect_options())
    try:
        LOGGER.info("Connected. Available resources: %s", machine.resource_names)
        arm = Arm.from_robot(machine, ARM_NAME)
        gripper = Gripper.from_robot(machine, GRIPPER_NAME)
        await pick_and_place(arm, gripper)
    finally:
        await machine.close()


if __name__ == "__main__":
    asyncio.run(main())
