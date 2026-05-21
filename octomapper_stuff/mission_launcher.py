#!/usr/bin/env python3

import asyncio
import os
import signal
from pathlib import Path

from drone_control import Drone


BASE_DIR = Path(__file__).resolve().parent

# Main scripts to run
OCTOMAP_LAUNCHER = BASE_DIR / "octomap_launcher.py"
OCTOMAP_TO_NUMPY = BASE_DIR / "octomap_to_numpy_grid.py"
FRONTIER_PATHFINDER = BASE_DIR / "frontier_pathfinder.py"


async def start_process(name, cmd, cwd=None):
    print(f"\nStarting {name}...")
    print(" ".join(str(c) for c in cmd))

    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"

    process = await asyncio.create_subprocess_exec(
        *cmd,
        cwd=str(cwd) if cwd else None,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE,
        env=env,
        preexec_fn=os.setsid,  # allows killing whole process group
    )

    async def print_stream(stream, prefix):
        while True:
            line = await stream.readline()
            if not line:
                break
            print(f"[{prefix}] {line.decode(errors='ignore').rstrip()}")

    asyncio.create_task(print_stream(process.stdout, name))
    asyncio.create_task(print_stream(process.stderr, name))

    return process


async def stop_process(name, process):
    if process is None or process.returncode is not None:
        return

    print(f"Stopping {name}...")

    try:
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
    except ProcessLookupError:
        return

    try:
        await asyncio.wait_for(process.wait(), timeout=5.0)
    except asyncio.TimeoutError:
        print(f"{name} did not stop cleanly. Killing...")
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        await process.wait()


async def main():
    processes = []
    drone = None
    drone_started = False

    try:
        # 1. Start OctoMap stack first
        # This should start:
        # - clock bridge
        # - fast_depth_to_pointcloud.py
        # - tf broadcaster
        # - octomap_server
        # - rviz2 if enabled
        octomap_process = await start_process(
            "octomap_launcher",
            ["python3", str(OCTOMAP_LAUNCHER)],
            cwd=BASE_DIR,
        )
        processes.append(("octomap_launcher", octomap_process))

        # Give OctoMap server and marker topics time to start
        await asyncio.sleep(5.0)

        numpy_grid_process = await start_process(
            "octomap_to_numpy_grid",
            ["python3", str(OCTOMAP_TO_NUMPY)],
            cwd=BASE_DIR,
        )
        processes.append(("octomap_to_numpy_grid", numpy_grid_process))

        # Give numpy grid/frontier visualisation time to start
        await asyncio.sleep(3.0)

        # 2. Connect and take off using your existing Drone class
        print("\nConnecting drone...")
        drone = Drone()
        await drone.connect()

        print("Arming and taking off...")
        await drone.arm_and_takeoff()
        drone_started = True

        print("Drone takeoff complete.")

        # 3. Start frontier pathfinder after drone is in offboard/takeoff state
        pathfinder_process = await start_process(
            "frontier_pathfinder",
            ["python3", str(FRONTIER_PATHFINDER)],
            cwd=BASE_DIR,
        )
        processes.append(("frontier_pathfinder", pathfinder_process))

        print("\nMission stack is running.")
        print("Press Ctrl+C to stop, land, and shut down.")

        while True:
            await asyncio.sleep(1.0)

    except KeyboardInterrupt:
        print("\nKeyboard interrupt received.")

    except Exception as e:
        print(f"\nMission launcher error: {type(e).name}: {e}")

    finally:
        print("\nShutting down mission...")

        # Stop pathfinder first so it stops sending new commands
        for name, process in reversed(processes):
            await stop_process(name, process)

        # Land after stopping external control processes
        if drone is not None and drone_started:
            try:
                print("Landing drone...")
                await drone.land()
            except Exception as e:
                print(f"Landing error: {type(e).name}: {e}")

        print("Mission shutdown complete.")


if __name__ == "__main__":
    asyncio.run(main())