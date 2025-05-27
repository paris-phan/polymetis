#!/usr/bin/env python3
import time
import threading
import logging
import torch
import pyspacemouse
from polymetis import RobotInterface
import math
# ─────────────────────────────────────────────────────────────────────────────
# Configuration: tuning knobs
HOST       = "192.168.1.15"      # or your robot IP
PORT       = "50051"
LIN_SENS   = 0.5              # meters per full SpaceMouse deflection
ANG_SENS   = 0.5              # radians per full deflection
SPM_MAX    = 2            # normalize raw SpaceMouse axis to [-1,1]
TOLERANCE  = 0.1
# ─────────────────────────────────────────────────────────────────────────────
# Shared SpaceMouse state
state = {"dx":0.0, "dy":0.0, "dz":0.0, "droll":0.0, "dpitch":0.0, "dyaw":0.0}
def spacemouse_reader():
    """Continuously read the SpaceMouse into `state`."""
    if not pyspacemouse.open():
        raise RuntimeError("Cannot open SpaceMouse!")
    while True:
        s = pyspacemouse.read()
        # normalize to [-1,1]
        state["dx"]    = s.y    / SPM_MAX
        state["dy"]    = s.x * -1   / SPM_MAX
        state["dz"]    = s.z    / SPM_MAX
        state["droll"] = s.roll / SPM_MAX
        state["dpitch"]= s.pitch/ SPM_MAX
        state["dyaw"]  = s.yaw  / SPM_MAX
        time.sleep(0.001)
        #print("reaedin")
def main():
    # 1) Start SpaceMouse thread
    t = threading.Thread(target=spacemouse_reader, daemon=True)
    t.start()
    # 2) Connect to Polymetis
    robot = RobotInterface(
                        ip_address=HOST,
                        port=PORT,
                    )
    hz    = robot.metadata.hz
    dt    = 1.0 / hz
    # 3) Cartesian impedance gains & launch controller
    Kx  = torch.Tensor(robot.metadata.default_Kx)
    Kxd = torch.Tensor(robot.metadata.default_Kxd)
    robot.start_cartesian_impedance(Kx=Kx, Kxd=Kxd)
    # 4) Grab the initial end-effector pose
    pos, ori = robot.get_ee_pose()
    # pos = torch.tensor([0.4, 0.0, 0.4])  # start at a fixed position
    # robot.update_desired_ee_pose(
    #                     position=pos + torch.tensor([0.0, 0.0, 0.0]),
    #                     orientation=ori
    #                 )
    # joint_positions_desired = torch.Tensor(
    #     [-0.14, -0.02, -0.05, -1.57, 0.05, 1.50, -1.91]
    # )
    # print(f"\nMoving joints to: {joint_positions_desired} ...\n")
    # state_log = robot.move_to_joint_positions(joint_positions_desired, time_to_go=2.0)
    time.sleep(1.0)
    print(f"Teleop running at {hz} Hz. Ctrl-C to exit.")
    try:
        while True:
            # compute small pose deltas
            dp   = torch.tensor([state["dx"], state["dy"], state["dz"]]) * LIN_SENS * dt
            drot = torch.tensor([state["droll"], state["dpitch"], state["dyaw"]]) * ANG_SENS * dt
            good = True
            #print("pos:", pos[0], pos[1], pos[2])
            if torch.norm(pos + dp) > (0.75 - TOLERANCE) or abs(pos[2] + dp[2]) < 0.00 + TOLERANCE:
                print("Out of bounds, adjusting...")
                print(f"pos is at {pos} and dp is {dp}")
                good = False
            #update position
            if good:
                pos = pos + dp
            dp = torch.tensor([state["dx"], state["dy"], state["dz"]]) * LIN_SENS * dt
            if abs(state["dx"])>0 or abs(state["dy"])>0 or abs(state["dz"])>0:
                print(f"POS : {pos}")
                #logging.INFO(f"{state['dx']:.3f}, dy={state['dy']:.3f}, dz={state['dz']:.3f}, droll={state['droll']:.3f}, dpitch={state['dpitch']:.3f}, dyaw={state['dyaw']:.3f},")
                print(f"{state['dx']:.3f}, dy={state['dy']:.3f}, dz={state['dz']:.3f}, droll={state['droll']:.3f}, dpitch={state['dpitch']:.3f}, dyaw={state['dyaw']:.3f},")
            # update orientation (via Euler → quaternion; you can swap in your math here)
            # euler = quat_to_euler(ori); euler += drot; ori = euler_to_quat(euler)
            # send desired pose
            robot.update_desired_ee_pose(position=pos, orientation=ori)
            #print("send to server")
            time.sleep(dt)
    except KeyboardInterrupt:
        print("\nShutting down teleop...")
        robot.terminate_current_policy()
if __name__ == "__main__":
    main()









