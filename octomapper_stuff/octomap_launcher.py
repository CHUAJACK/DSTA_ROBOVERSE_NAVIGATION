#!/usr/bin/env python3

import asyncio
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent

POINTCLOUD_TOPIC = "/depth_camera/points_fast"
CAMERA_FRAME = "camera_link"
RESOLUTION = "0.5"


async def start_process(name, cmd):
    print(f"\nStarting {name}...")
    print(" ".join(cmd))

    process = await asyncio.create_subprocess_exec(
        *cmd,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE,
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
    process.terminate()

    try:
        await asyncio.wait_for(process.wait(), timeout=5.0)
    except asyncio.TimeoutError:
        print(f"{name} did not stop cleanly. Killing...")
        process.kill()
        await process.wait()


async def main():
    processes = []

    try:
        # 1. Bridge Gazebo /clock to ROS 2 /clock
        clock_bridge_cmd = [
            "ros2", "run", "ros_gz_bridge", "parameter_bridge",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ]

        clock_bridge = await start_process("clock_bridge", clock_bridge_cmd)
        processes.append(("clock_bridge", clock_bridge))

        await asyncio.sleep(1.0)

        # 2. Start fast depth-image to PointCloud2 converter
        fast_pc_cmd = [
            "python3",
            str(BASE_DIR / "fast_depth_to_pointcloud.py"),
        ]

        fast_pc = await start_process("fast_pointcloud_generator", fast_pc_cmd)
        processes.append(("fast_pointcloud_generator", fast_pc))

        await asyncio.sleep(2.0)

        # 3. Start MAVSDK dynamic TF broadcaster
        tf_cmd = [
            "python3",
            str(BASE_DIR / "mavsdk_body_camera_tf_broadcaster.py"),
        ]

        tf = await start_process("tf_broadcaster", tf_cmd)
        processes.append(("tf_broadcaster", tf))

        await asyncio.sleep(5.0)

        # 4. Start OctoMap server with sim time enabled
        octomap_cmd = [
            "ros2", "run", "octomap_server", "octomap_server_node",
            "--ros-args",
            "-r", f"cloud_in:={POINTCLOUD_TOPIC}",
            "-p", "frame_id:=map",
            "-p", f"resolution:={RESOLUTION}",
            "-p", "use_sim_time:=true",
            "-p", "publish_free_space:=true",
        ]

        octomap = await start_process("octomap_server", octomap_cmd)
        processes.append(("octomap_server", octomap))

        await asyncio.sleep(2.0)

        # 5. Optional RViz2 with sim time enabled
        rviz_config = BASE_DIR / "octomap_frontier.rviz"

        rviz_cmd = [
            "rviz2",
            "-d", str(rviz_config),
            "--ros-args",
            "-p", "use_sim_time:=true",
        ]

        rviz = await start_process("rviz2", rviz_cmd)
        processes.append(("rviz2", rviz))

        print("\nOctoMap stack is running with sim time.")
        print("Press Ctrl+C to stop all processes.")

        while True:
            await asyncio.sleep(1.0)

    except KeyboardInterrupt:
        print("\nKeyboard interrupt received.")

    finally:
        print("\nShutting down OctoMap stack...")

        for name, process in reversed(processes):
            await stop_process(name, process)

        print("Shutdown complete.")


if __name__ == "__main__":
    asyncio.run(main())