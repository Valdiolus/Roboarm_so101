#!/usr/bin/env python3
import math
import time

from lerobot.robots.so_follower import SO101FollowerConfig, SO101Follower


PORT = "/dev/tty.usbmodem5AAF2197991"
ARM_ID = "valdis_new_gripper_follower_arm"

# Neutral pose for all joints before the shoulder-pan test motion.
READY_POSE = {
    "shoulder_pan.pos": 0.0,
    "shoulder_lift.pos": 0.0,
    "elbow_flex.pos": 0.0,
    "wrist_flex.pos": 0.0,
    "wrist_roll.pos": 0.0,
    "gripper.pos": 0.0,
}

REST_POSE = {
    "shoulder_pan.pos": 0.0,
    "shoulder_lift.pos": -100.0,
    "elbow_flex.pos": 100.0,
    "wrist_flex.pos": 50.0,
    "wrist_roll.pos": 0.0,
    "gripper.pos": 0.0,
}
def move_to_ready_pose(robot):
    print("Moving to initial joint positions...")
    for _ in range(20):
        robot.send_action(READY_POSE)
        time.sleep(0.05)

def move_to_rest_pose(robot):
    print("Moving to rest joint positions...")
    for _ in range(20):
        robot.send_action(REST_POSE)
        time.sleep(0.05)

def main():
    config = SO101FollowerConfig(port=PORT, id=ARM_ID)
    robot = SO101Follower(config)
    
    robot.connect()
    move_to_ready_pose(robot)
    #time.sleep(10.0)

    try:

        freq_hz = 1.2
        amplitude_deg = 6.0
        center_deg = 0.0
        period = 1.0 / freq_hz
        start_time = time.monotonic()

        print(
            f"Starting shoulder_pan oscillation: center={center_deg}deg, amplitude={amplitude_deg}deg, "
            f"freq={freq_hz}Hz"
        )

        while True:
            elapsed = time.monotonic() - start_time
            phase = elapsed / period
            shoulder_pan = center_deg + amplitude_deg * math.sin(2.0 * math.pi * phase)

            action = dict(READY_POSE)
            action["shoulder_pan.pos"] = shoulder_pan
            robot.send_action(action)

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopping follower test.")
    finally:
        try:
            move_to_rest_pose(robot)
            time.sleep(1.0)
            robot.disconnect()
        except Exception:
            pass
        print("Robot disconnected.")


if __name__ == "__main__":
    main()
