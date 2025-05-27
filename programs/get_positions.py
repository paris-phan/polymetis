import torch
import time
from polymetis import RobotInterface

# ─────────────────────────────────────────────────────────────────────────────
# Configuration: tuning knobs
HOST       = "192.168.1.15"      # or your robot IP
PORT       = "50051"
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    # Initialize robot interface
    robot = RobotInterface(
        ip_address=HOST,
        port=PORT
    )
#Get joint positions
joint_positions = robot.get_joint_positions()
print(f"Current joint positions: {joint_positions}")
#Get pos, and orientation
pos, ori = robot.get_ee_pose()
print(f"Current end-effector position: {pos}")
print(f"Current end-effector orientation: {ori}")