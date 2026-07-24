"""Basic pick-and-place example for the Viam Python SDK.

The robot picks an object from one location and places it at another.
Motion is planned through the Viam `motion` service so the arm avoids
collisions with obstacles you declare in the world state (e.g. the table).

Configure your machine's API key, address, and component names via
environment variables (see `.env.example`), then run:

    python pick_and_place.py

Requires an arm, a gripper, and the built-in motion service configured on
your machine. Update the pick/place poses below to match your workspace.
"""

import asyncio
import os

from viam.components.arm import Arm
from viam.components.gripper import Gripper
from viam.logging import getLogger
from viam.proto.common import (
    Geometry,
    GeometriesInFrame,
    Pose,
    PoseInFrame,
    RectangularPrism,
    Vector3,
    WorldState,
)
from viam.robot.client import RobotClient
from viam.services.motion import MotionClient

LOGGER = getLogger(__name__)

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
# Credentials come from environment variables so they never live in source.
# Get these from your machine's CONNECT tab in the Viam app.
API_KEY = os.environ.get("VIAM_API_KEY", "")
API_KEY_ID = os.environ.get("VIAM_API_KEY_ID", "")
MACHINE_ADDRESS = os.environ.get("VIAM_MACHINE_ADDRESS", "")

# Resource names as configured on your machine.
ARM_NAME = os.environ.get("VIAM_ARM_NAME", "arm")
GRIPPER_NAME = os.environ.get("VIAM_GRIPPER_NAME", "gripper")
MOTION_NAME = os.environ.get("VIAM_MOTION_NAME", "builtin")

# All positions are in millimeters, relative to the "world" reference frame.
# Orientation is a Viam orientation vector; (o_x, o_y, o_z) = (0, 0, -1) points
# the gripper straight down, which is the usual approach for top-down picking.
#
# ⚠️  These are placeholder coordinates. Measure your own workspace and update
# them before running against real hardware, or you risk driving the arm into
# something.
GRIPPER_DOWN = {"o_x": 0.0, "o_y": 0.0, "o_z": -1.0, "theta": 0.0}

PICK_POSE = Pose(x=400.0, y=0.0, z=100.0, **GRIPPER_DOWN)
PLACE_POSE = Pose(x=400.0, y=300.0, z=100.0, **GRIPPER_DOWN)

# How far above pick/place poses to approach from and retreat to (mm).
APPROACH_HEIGHT = 100.0


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


def build_world_state() -> WorldState:
    """Declare static obstacles so the motion planner avoids them.

    Here we model the table as a flat box just below the workspace. Add more
    geometries (bins, walls, fixtures) to reflect your real environment.
    """
    table = Geometry(
        center=Pose(x=400.0, y=150.0, z=-10.0, o_x=0.0, o_y=0.0, o_z=1.0, theta=0.0),
        box=RectangularPrism(dims_mm=Vector3(x=1000.0, y=1000.0, z=20.0)),
        label="table",
    )
    obstacles = GeometriesInFrame(reference_frame="world", geometries=[table])
    return WorldState(obstacles=[obstacles])


def above(pose: Pose, height: float) -> Pose:
    """Return a copy of `pose` raised by `height` mm along z."""
    return Pose(
        x=pose.x,
        y=pose.y,
        z=pose.z + height,
        o_x=pose.o_x,
        o_y=pose.o_y,
        o_z=pose.o_z,
        theta=pose.theta,
    )


async def move_arm_to(
    motion: MotionClient,
    arm_resource_name,
    pose: Pose,
    world_state: WorldState,
    description: str,
) -> None:
    """Plan and execute a collision-free move of the arm to `pose`."""
    LOGGER.info("Moving arm to %s: %s", description, pose)
    destination = PoseInFrame(reference_frame="world", pose=pose)
    moved = await motion.move(
        component_name=arm_resource_name,
        destination=destination,
        world_state=world_state,
    )
    if not moved:
        raise RuntimeError(f"Motion planner failed to move arm to {description}")


async def pick_and_place(
    motion: MotionClient,
    gripper: Gripper,
    world_state: WorldState,
) -> None:
    """Run one full pick-and-place cycle."""
    arm_resource_name = Arm.get_resource_name(ARM_NAME)

    pick_approach = above(PICK_POSE, APPROACH_HEIGHT)
    place_approach = above(PLACE_POSE, APPROACH_HEIGHT)

    # --- Pick ---
    LOGGER.info("Opening gripper before pick")
    await gripper.open()

    await move_arm_to(motion, arm_resource_name, pick_approach, world_state, "pick approach")
    await move_arm_to(motion, arm_resource_name, PICK_POSE, world_state, "pick pose")

    LOGGER.info("Grabbing object")
    grabbed = await gripper.grab()
    if not grabbed:
        LOGGER.warning("Gripper reported it did not grab anything")

    await move_arm_to(motion, arm_resource_name, pick_approach, world_state, "pick retreat")

    # --- Place ---
    await move_arm_to(motion, arm_resource_name, place_approach, world_state, "place approach")
    await move_arm_to(motion, arm_resource_name, PLACE_POSE, world_state, "place pose")

    LOGGER.info("Releasing object")
    await gripper.open()

    await move_arm_to(motion, arm_resource_name, place_approach, world_state, "place retreat")

    LOGGER.info("Pick-and-place cycle complete")


async def main() -> None:
    machine = await RobotClient.at_address(MACHINE_ADDRESS, connect_options())
    try:
        LOGGER.info("Connected. Available resources: %s", machine.resource_names)

        gripper = Gripper.from_robot(machine, GRIPPER_NAME)
        motion = MotionClient.from_robot(machine, MOTION_NAME)
        world_state = build_world_state()

        await pick_and_place(motion, gripper, world_state)
    finally:
        await machine.close()


if __name__ == "__main__":
    asyncio.run(main())
