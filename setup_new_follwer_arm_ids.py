# set_follower_ids.py

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

PORT = "/dev/tty.usbmodem5AB01815731"   # change this

motors = {
    "shoulder_pan":  Motor(7,  "sts3215", MotorNormMode.DEGREES),
    "shoulder_lift": Motor(8,  "sts3215", MotorNormMode.DEGREES),
    "elbow_flex":    Motor(9,  "sts3215", MotorNormMode.DEGREES),
    "wrist_flex":    Motor(10, "sts3215", MotorNormMode.DEGREES),
    "wrist_roll":    Motor(11, "sts3215", MotorNormMode.DEGREES),
    "gripper":       Motor(12, "sts3215", MotorNormMode.RANGE_0_100),
}

bus = FeetechMotorsBus(
    port=PORT,
    motors=motors,
)

# We deliberately do one servo at a time.
for name in [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]:
    target_id = motors[name].id

    print()
    print(f"=== {name}: target ID {target_id} ===")
    input(
        f"Connect ONLY follower '{name}' motor to the controller, "
        "then press ENTER..."
    )

    bus.setup_motor(name)

    print(f"SUCCESS: {name} -> ID {target_id}")

print("\nFollower IDs configured: 7,8,9,10,11,12")