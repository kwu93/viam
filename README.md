# Viam Pick-and-Place Example

Starter code for a basic pick-and-place routine using the [Viam Python SDK](https://github.com/viamrobotics/viam-python-sdk).
The robot picks an object from one location and places it at another, planning motion through Viam's `motion` service so the arm avoids obstacles you declare.

## What it does

`pick_and_place.py` runs a single pick-and-place cycle:

1. Open the gripper.
2. Move to an approach pose above the pick location.
3. Lower to the pick pose and grab the object.
4. Retreat straight up.
5. Move to an approach pose above the place location.
6. Lower to the place pose and release the object.
7. Retreat straight up.

Each move is planned by the `motion` service against a world state that includes the table as an obstacle, so the planner routes around it.

## Prerequisites

- Python 3.9 or newer.
- A Viam machine (real or simulated) with these components configured:
  - an **arm** (default name `arm`),
  - a **gripper** (default name `gripper`),
  - the built-in **motion** service (default name `builtin`).
- An API key for the machine, from the machine's **CONNECT** tab at [app.viam.com](https://app.viam.com).

If you do not have hardware yet, you can configure a [fake arm](https://docs.viam.com/components/arm/fake/) and [fake gripper](https://docs.viam.com/components/gripper/fake/) to try the flow end to end in simulation.

## Setup

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Copy the example environment file and fill in your machine's credentials:

```bash
cp .env.example .env
# edit .env, then load it:
set -a; source .env; set +a
```

## Run

```bash
python pick_and_place.py
```

## Configure for your workspace

The pick and place coordinates in `pick_and_place.py` are **placeholders**.
Positions are in millimeters relative to the `world` reference frame, and orientation uses a Viam orientation vector (with `(o_x, o_y, o_z) = (0, 0, -1)` pointing the gripper straight down for top-down picking).

Before running against real hardware, update these to match your setup:

- `PICK_POSE` - where the object starts.
- `PLACE_POSE` - where the object should end up.
- `APPROACH_HEIGHT` - how far above each pose to approach from and retreat to.
- `build_world_state()` - the obstacles the planner must avoid. The example models only a table; add bins, walls, or fixtures to match your real environment.

> **Safety:** wrong coordinates or a missing obstacle can drive the arm into something.
> Verify your poses at low speed, and keep an emergency stop within reach when testing on hardware.

## Learn more

- [Viam Python SDK docs](https://python.viam.dev/)
- [Motion service](https://docs.viam.com/services/motion/)
- [Arm component](https://docs.viam.com/components/arm/)
- [Gripper component](https://docs.viam.com/components/gripper/)
