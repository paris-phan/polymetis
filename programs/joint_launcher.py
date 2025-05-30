#!/usr/bin/env python3
import time
import threading
import logging
import torch
import pyspacemouse
from polymetis import RobotInterface, GripperInterface
import math

# Robot and gripper configuration
HOST = "192.168.1.15"
PORT = "50051"
GRIPPER_PORT = "50052"
LIN_SENS = 0.5
ANG_SENS = 0.5
SPM_MAX = 2
TOLERANCE = 0.1

# Gripper settings
GRIPPER_SPEED = 0.1
GRIPPER_FORCE = 0.1
GRIPPER_WIDTH_OPEN = 0.085
GRIPPER_WIDTH_CLOSED = 0.0

state = {"dx": 0.0, "dy": 0.0, "dz": 0.0, "droll": 0.0, "dpitch": 0.0, "dyaw": 0.0}

def spacemouse_reader():
    if not pyspacemouse.open():
        raise RuntimeError("Cannot open SpaceMouse!")
    while True:
        s = pyspacemouse.read()
        state["dx"] = s.y / SPM_MAX
        state["dy"] = s.x * -1 / SPM_MAX
        state["dz"] = s.z / SPM_MAX
        state["droll"] = s.roll / SPM_MAX
        state["dpitch"] = s.pitch / SPM_MAX
        state["dyaw"] = s.yaw / SPM_MAX
        time.sleep(0.001)

def main():
    # Initialize SpaceMouse
    t = threading.Thread(target=spacemouse_reader, daemon=True)
    t.start()

    # Connect to robot and gripper
    robot = RobotInterface(ip_address=HOST, port=PORT)
    gripper = GripperInterface(ip_address=HOST, port=GRIPPER_PORT)

    hz = robot.metadata.hz
    dt = 1.0 / hz

    # Start impedance control
    Kx = torch.Tensor(robot.metadata.default_Kx)
    Kxd = torch.Tensor(robot.metadata.default_Kxd)
    robot.start_cartesian_impedance(Kx=Kx, Kxd=Kxd)

    pos, ori = robot.get_ee_pose()
    time.sleep(1.0)

    print(f"Teleop running at {hz} Hz. Ctrl-C to exit.")
    print("Controls:")
    print("- SpaceMouse: Control robot position/orientation")
    print("- Left button: Open gripper")
    print("- Right button: Close gripper")

    try:
        while True:
            # Gripper control
            if pyspacemouse.read().buttons[0]:
                gripper.goto(GRIPPER_WIDTH_OPEN, GRIPPER_SPEED, GRIPPER_FORCE)
            elif pyspacemouse.read().buttons[1]:
                gripper.goto(GRIPPER_WIDTH_CLOSED, GRIPPER_SPEED, GRIPPER_FORCE)

            # Robot control
            dp = torch.tensor([state["dx"], state["dy"], state["dz"]]) * LIN_SENS * dt
            drot = torch.tensor([state["droll"], state["dpitch"], state["dyaw"]]) * ANG_SENS * dt
            good = True

            if torch.norm(pos + dp) > (0.75 - TOLERANCE) or abs(pos[2] + dp[2]) < 0.00 + TOLERANCE:
                print("Out of bounds, adjusting...")
                print(f"pos is at {pos} and dp is {dp}")
                good = False

            if good:
                pos = pos + dp

            if abs(state["dx"]) > 0 or abs(state["dy"]) > 0 or abs(state["dz"]) > 0:
                print(f"POS : {pos}")
                print(f"{state['dx']:.3f}, dy={state['dy']:.3f}, dz={state['dz']:.3f}, droll={state['droll']:.3f}, dpitch={state['dpitch']:.3f}, dyaw={state['dyaw']:.3f},")

            robot.update_desired_ee_pose(position=pos, orientation=ori)
            time.sleep(dt)

    except KeyboardInterrupt:
        print("\nShutting down teleop...")
        robot.terminate_current_policy()

if __name__ == "__main__":
    main()
