#!/usr/bin/env python3
import math
import time

from lerobot.robots.so_follower import SO101FollowerConfig, SO101Follower


PORT = "/dev/ttyACM0"
ARM_ID = "valdis_new_gripper_follower_arm"

# Neutral pose for all joints before the shoulder-pan test motion.
READY_POSE = {
    "shoulder_pan.pos": 0.0,
    "shoulder_lift.pos": 0.0,
    "elbow_flex.pos": 0.0,
    "wrist_flex.pos": -30.0,
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

    # Oscillation parameters per joint: (amplitude_deg, freq_hz)
    JOINT_OSCILLATIONS = {
        "shoulder_pan":  (6.0,  1.2),
        "shoulder_lift": (4.0,  2.4),
        "elbow_flex":    (6.0,  2.4),  # opposite direction via negative amplitude
        "wrist_flex":    (3.0,  2.4),
        "wrist_roll":    (4.0,  1.2),
    }

    start_time = time.monotonic()
    print("Starting multi-joint oscillations:")
    for joint, (amp, freq) in JOINT_OSCILLATIONS.items():
        print(f"  {joint}: ±{amp}deg at {freq}Hz")

    # Forward-backward movement parameters (single 10 Hz cycle, 0.1s duration).
    DASH_amplitude_deg = {
        "shoulder_lift":  15.0,
        "elbow_flex":    -15.0,
        "wrist_flex":   -10.0,
    }
    DASH_FREQ_HZ = 2.0
    DASH_MIN_PERIOD = 0.5  # min time between DASH triggers
    DASH_MAX_PERIOD = 2  # max time between DASH triggers

    # Heavy impact parameters: impact phase 5 Hz, recovery phase 1 Hz.
    # Amplitude ranges per joint (min_deg, max_deg).
    IMPACT_AMPLITUDE_RANGES = {
        "shoulder_pan": (6.0, 10.0),
        "shoulder_lift": (3.0, 5.0),
        "elbow_flex": (4.0, 7.0),
        "wrist_flex": (3.0, 5.0),
    }
    IMPACT_FREQ_HZ = 5.0   # impact phase frequency
    RECOVERY_FREQ_HZ = 1.0 # recovery phase frequency
    IMPACT_RATIO = 0.3     # fraction of cycle spent in impact (0.3 = 30% impact, 70% recovery)
    IMPACT_MIN_PERIOD = 1.0  # min time between impact triggers
    IMPACT_MAX_PERIOD = 5.0  # max time between impact triggers

    try:
        # Track the last completed period index per joint for randomization resets.
        joint_last_period = {}
        joint_random_amp = {}

        # Random forward-backward movement tracking.
        import random
        next_DASH_time = None  # absolute time when next DASH triggers
        DASH_cycle_start = None  # when current DASH cycle started (None = idle)
        DASH_elapsed = 0.0  # elapsed time within current DASH cycle
        dash_in_progress = False  # flag: True while DASH cycle is active

        # Heavy impact tracking.
        next_impact_time = None  # absolute time when next impact triggers
        impact_cycle_start = None  # when current impact cycle started (None = idle)
        impact_in_progress = False  # flag: True while impact cycle is active

        while True:
            elapsed = time.monotonic() - start_time
            action = dict(READY_POSE)

            for joint, (amplitude_deg, freq_hz) in JOINT_OSCILLATIONS.items():
                period = 1.0 / freq_hz
                phase = elapsed / period
                current_period = int(phase)

                # New period → draw a fresh random amplitude (±30% of base).
                if joint not in joint_last_period or current_period != joint_last_period[joint]:
                    joint_random_amp[joint] = amplitude_deg * (1.0 + random.uniform(-0.30, 0.30))
                    joint_last_period[joint] = current_period
                    print(f"[{joint}] period {current_period} → amp={joint_random_amp[joint]:.2f}deg (base={amplitude_deg}deg)")

                # Negative amplitude for elbow_flex gives opposite direction to shoulder_lift
                sign = -1.0 if joint == "elbow_flex" else 1.0
                offset = sign * joint_random_amp[joint] * math.sin(2.0 * math.pi * phase)
                base = READY_POSE[f"{joint}.pos"]
                action[f"{joint}.pos"] = round(base + offset, 2)

            # --- Forward-backward random DASH on top of oscillations ---
            if next_DASH_time is None:
                # Schedule next DASH 2-5 seconds from now.
                next_DASH_time = elapsed + random.uniform(DASH_MIN_PERIOD, DASH_MAX_PERIOD)

            if DASH_cycle_start is None:
                # Check if it's time to start a DASH cycle.
                if elapsed >= next_DASH_time:
                    DASH_cycle_start = elapsed
                    DASH_elapsed = 0.0
                    dash_in_progress = True
                    print(f"[DASH] triggering forward-backward at t={elapsed:.2f}s (IN_PROGRESS)")

            if DASH_cycle_start is not None:
                # DASH cycle is active — compute offset for 10 Hz single period.
                DASH_elapsed = elapsed - DASH_cycle_start
                DASH_period = 1.0 / DASH_FREQ_HZ  # 0.1s

                if DASH_elapsed <= DASH_period:
                    # Within the 0.1s cycle — apply sine-based offsets.
                    DASH_phase = DASH_elapsed / DASH_period  # 0→1 over the cycle
                    for joint, DASH_amp in DASH_amplitude_deg.items():
                        DASH_offset = DASH_amp * math.sin(2.0 * math.pi * DASH_phase)
                        action[f"{joint}.pos"] = round(action[f"{joint}.pos"] + DASH_offset, 2)
                else:
                    # DASH cycle finished — reset for next random trigger.
                    DASH_cycle_start = None
                    next_DASH_time = None
                    dash_in_progress = False
                    print(f"[DASH] completed at t={elapsed:.2f}s (IN_PROGRESS=False)")

            # --- Heavy impact on top of oscillations + DASH ---
            if next_impact_time is None:
                # Schedule next impact 1-5 seconds from now.
                next_impact_time = elapsed + random.uniform(IMPACT_MIN_PERIOD, IMPACT_MAX_PERIOD)

            if impact_cycle_start is None:
                # Check if it's time to start an impact cycle.
                if elapsed >= next_impact_time:
                    impact_cycle_start = elapsed
                    impact_in_progress = True
                    print(f"[IMPACT] triggering heavy impact at t={elapsed:.2f}s (IN_PROGRESS)")

            if impact_cycle_start is not None:
                # Impact cycle is active — two-phase: impact (5 Hz) then recovery (1 Hz).
                impact_elapsed = elapsed - impact_cycle_start
                total_cycle = 1.0 / RECOVERY_FREQ_HZ  # 1.0s total cycle (at recovery speed)
                impact_duration = IMPACT_RATIO * total_cycle  # 0.3s impact phase
                recovery_duration = total_cycle - impact_duration  # 0.7s recovery phase

                if impact_elapsed < total_cycle:
                    # Determine which phase we're in.
                    if impact_elapsed < impact_duration:
                        # Impact phase: 5 Hz, full amplitudes.
                        impact_phase = impact_elapsed / impact_duration  # 0→1 over impact
                        impact_sine = math.sin(2.0 * math.pi * impact_phase)
                        phase_freq = IMPACT_FREQ_HZ
                    else:
                        # Recovery phase: 1 Hz, gradually dampened.
                        recovery_phase = (impact_elapsed - impact_duration) / recovery_duration  # 0→1 over recovery
                        impact_sine = math.sin(2.0 * math.pi * recovery_phase)
                        phase_freq = RECOVERY_FREQ_HZ

                    # Apply impact offsets for each joint.
                    for joint, (amp_min, amp_max) in IMPACT_AMPLITUDE_RANGES.items():
                        # Random amplitude chosen once per impact event.
                        amp = random.uniform(amp_min, amp_max)
                        impact_offset = amp * impact_sine
                        action[f"{joint}.pos"] = round(action[f"{joint}.pos"] + impact_offset, 2)

                    print(f"[IMPACT] t={impact_elapsed:.3f}s phase={'impact' if impact_elapsed < impact_duration else 'recovery'} sine={impact_sine:+.3f}")
                else:
                    # Impact cycle finished — reset for next random trigger.
                    impact_cycle_start = None
                    next_impact_time = None
                    impact_in_progress = False
                    print(f"[IMPACT] completed at t={elapsed:.2f}s (IN_PROGRESS=False)")

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
