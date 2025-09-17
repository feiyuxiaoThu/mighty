#!/usr/bin/env python3
# File: launch_sim.py
import subprocess
import sys
import argparse

def run_tmux(cmd):
    """Run a tmux command and exit if an error occurs."""
    try:
        subprocess.run(cmd, shell=True, check=True)
    except subprocess.CalledProcessError:
        print(f"Error executing: {cmd}")
        sys.exit(1)

def main():
    parser = argparse.ArgumentParser(
        description="Launch multiagent simulation tmux session with a specified environment"
    )
    parser.add_argument(
        "--env",
        type=str,
        default="multiagent_testing",
        help="Environment name (must be one of env_cmds keys)",
    )
    parser.add_argument(
        "--namespaces",
        nargs="+",
        default=["NX01"],
        help="Agent namespaces to launch (e.g. NX01 NX02 NX03)",
    )
    args = parser.parse_args()
    env_name = args.env
    namespaces = args.namespaces

    env_cmds = {
        "multiagent_testing": "ros2 launch dynus onboard_dynus.launch.py x:={x} y:={y} z:={z} yaw:={yaw} namespace:={namespace} use_obstacle_tracker:=true depth_camera_name:=d435",
    }

    if env_name not in env_cmds:
        print(f"Error: Environment '{env_name}' not recognized.")
        print("Available environments:")
        for key in env_cmds:
            print("  -", key)
        sys.exit(1)

    # Common setup for every pane
    setup_cmd = "source /home/kkondo/code/dynus_ws/install/setup.bash; export ROS_DOMAIN_ID=7;"

    # Figure out if we need obstacle tracker for the base station
    use_dyn_obs = "true" if env_name in ["empty_wo_ground", "empty", "dynamic_debug"] else "false"

    # Handle any env_name remapping (dynamic_debug, quadruped, etc.)
    # … keep your existing remapping logic here if still needed …

    # Base‐station launch
    base_station_cmd = (
        f"{setup_cmd} "
        "ros2 launch dynus base_dynus.launch.py "
        f"use_dyn_obs:={use_dyn_obs} "
        "use_gazebo_gui:=false "
        "use_rviz:=true "
        "use_ground_robot:=false "
        "benchmark_name:=benchmark2 "
        f"env:={env_name}"
    )

    session_name = "multiagent_sim"
    window_name = "main"

    # Kill any stale session
    subprocess.run(f"tmux kill-session -t {session_name}", shell=True, stderr=subprocess.DEVNULL)

    # Create new session & first window
    run_tmux(f"tmux new-session -d -s {session_name} -n {window_name}")

    # Pane 0: base station
    run_tmux(f"tmux send-keys -t {session_name}:0.0 '{base_station_cmd}' C-m")

    # Initial position map for each namespace
    initial_positions = {
        "NX01": {"x": 0.0, "y": 3.0, "z": 2.0, "yaw": 0.0},
        "NX02": {"x": -3.0, "y": 1.5, "z": 2.0, "yaw": 0.0},
        "NX03": {"x": -3.0, "y": -1.5, "z": 2.0, "yaw": 0.0},
        "NX04": {"x": 0.0, "y": -3.0, "z": 2.0, "yaw": 0.0},
        # Add more namespaces as needed
    }

    # For each agent namespace, spin up one pane
    # assume pane 0 is your base station
    template = env_cmds[env_name]

    for idx, ns in enumerate(namespaces):
        coords = initial_positions.get(ns, {"x":0.0,"y":0.0,"z":3.0,"yaw":0.0})
        ros_launch = template.format(
            x=coords["x"], y=coords["y"], z=coords["z"], yaw=coords["yaw"], namespace=ns
        )

        sleep_time = 10 + idx * 5  # staggered sleep for each agent
        full_cmd = f"{setup_cmd} sleep {sleep_time}; {ros_launch}"

        # split & immediately run your ros2 launch in the new pane
        run_tmux(f"tmux split-window -h -t {session_name}:0")
        run_tmux(f"tmux send-keys -t {session_name}:0.{idx + 1} '{full_cmd}' C-m")



    # If you still want cave‐specific object detection in its own pane:
    if env_name.startswith("cave_"):
        obj_det_cmd = f"{setup_cmd} conda activate yolo && ros2 launch dynus object_detection.launch.py"
        run_tmux(f"tmux split-window -t {session_name}:0 '{obj_det_cmd}'")

    # Finally tile them all and attach
    run_tmux(f"tmux select-layout -t {session_name}:0 tiled")
    run_tmux(f"tmux attach-session -t {session_name}")

if __name__ == "__main__":
    main()
