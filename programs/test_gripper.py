#!/usr/bin/env python3
import time
import logging
from polymetis import GripperInterface

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def test_gripper():
    logger.info("Initializing gripper interface...")
    gripper = GripperInterface(
        ip_address="localhost",
        port=50052
    )

    try:
        initial_state = gripper.get_state()
        logger.info(f"Initial gripper state: {initial_state}")

        logger.info("Starting gripper test sequence...")

        logger.info("Opening gripper...")
        gripper.goto(width=0.08, speed=0.1, force=50.0)
        time.sleep(2.0)

        logger.info("Closing gripper...")
        gripper.goto(width=0.0, speed=0.1, force=50.0)
        time.sleep(2.0)

        logger.info("Testing grasp...")
        gripper.grasp(
            speed=0.1,
            force=50.0,
            grasp_width=0.04,
            epsilon_inner=0.01,
            epsilon_outer=0.01
        )
        time.sleep(2.0)

        logger.info("Opening gripper again...")
        gripper.goto(width=0.08, speed=0.1, force=50.0)
        time.sleep(2.0)

        final_state = gripper.get_state()
        logger.info(f"Final gripper state: {final_state}")

    except Exception as e:
        logger.error(f"Error during gripper test: {e}")
        raise

    logger.info("Gripper test sequence completed!")

if __name__ == "__main__":
    test_gripper() 