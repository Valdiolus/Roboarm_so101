#!/usr/bin/env python3
import math
import time
import os

import pyrealsense2 as rs
import numpy as np
import cv2

from hailo_platform import (
    HEF,
    VDevice,
    FormatType,
    HailoSchedulingAlgorithm,
)
from lerobot.robots.so_follower import SO101FollowerConfig, SO101Follower

HEF_PATH = "yolov8s.hef"
CONF_THRESHOLD = 0.35
PAN_TRACKING_STEP_DEG = 1.0


PORT = "/dev/ttyACM0"
ARM_ID = "valdis_new_gripper_follower_arm"

# Camera configuration
CAM_WIDTH = 640
CAM_HEIGHT = 480
CAM_FPS = 60
CAM_JPEG_PATH = "cam.jpg"
VIDEO_MP4_PATH = "video.mp4"
VIDEO_FPS = 30  # FPS for the output video file (mp4)

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


def letterbox_rgb(image_rgb, dst_w, dst_h):
    """Resize an RGB image with aspect-ratio-preserving padding."""
    src_h, src_w = image_rgb.shape[:2]
    scale = min(dst_w / src_w, dst_h / src_h)
    new_w = int(src_w * scale)
    new_h = int(src_h * scale)

    resized = cv2.resize(
        image_rgb,
        (new_w, new_h),
        interpolation=cv2.INTER_LINEAR,
    )

    output = np.full((dst_h, dst_w, 3), 114, dtype=np.uint8)
    pad_x = (dst_w - new_w) // 2
    pad_y = (dst_h - new_h) // 2
    output[pad_y : pad_y + new_h, pad_x : pad_x + new_w] = resized

    return output, scale, pad_x, pad_y


class HailoYOLO:
    """Hailo YOLO inference wrapper for object detection."""

    def __init__(self, hef_path):
        # Hailo device configuration
        params = VDevice.create_params()
        params.scheduling_algorithm = HailoSchedulingAlgorithm.ROUND_ROBIN
        params.group_id = "SHARED"

        # Initialize Hailo device
        self.device = VDevice(params)

        # Load HEF
        self.hef = HEF(hef_path)

        input_info = self.hef.get_input_vstream_infos()[0]
        self.input_h = input_info.shape[0]
        self.input_w = input_info.shape[1]

        print(
            f"Model input: "
            f"{self.input_w}x{self.input_h}"
        )

        # Create inference model
        self.model = self.device.create_infer_model(hef_path)
        self.model.set_batch_size(1)

        # We send uint8 RGB frames and receive float NMS detections.
        self.model.input().set_format_type(FormatType.UINT8)
        self.output_names = []

        for output_info in self.hef.get_output_vstream_infos():
            name = output_info.name
            self.output_names.append(name)
            self.model.output(name).set_format_type(
                FormatType.FLOAT32
            )
            print("Output:", name, output_info.shape)

        # Configure the network once and retain the active context.
        self.config_context = self.model.configure()
        self.configured_model = self.config_context.__enter__()

    def infer(self, image_rgb):
        """Run inference on a uint8 HxWx3 RGB image."""
        processed, scale, pad_x, pad_y = letterbox_rgb(
            image_rgb,
            self.input_w,
            self.input_h,
        )

        output_buffers = {
            name: np.empty(self.model.output(name).shape, dtype=np.float32)
            for name in self.output_names
        }
        bindings = self.configured_model.create_bindings(
            output_buffers=output_buffers
        )
        bindings.input().set_buffer(processed)

        errors = []

        def callback(completion_info):
            if completion_info.exception is not None:
                errors.append(completion_info.exception)

        self.configured_model.wait_for_async_ready(timeout_ms=10000)
        job = self.configured_model.run_async([bindings], callback)
        job.wait(10000)

        if errors:
            raise RuntimeError(errors[0])

        if len(self.output_names) == 1:
            output = bindings.output().get_buffer()
        else:
            output = {
                name: bindings.output(name).get_buffer()
                for name in self.output_names
            }

        return decode_hailo_nms(
            output,
            image_rgb.shape,
            self.input_w,
            self.input_h,
            scale,
            pad_x,
            pad_y,
        )

    def close(self):
        """Clean up resources."""
        if self.config_context is not None:
            self.config_context.__exit__(None, None, None)


def decode_hailo_nms(
    output,
    image_shape,
    model_w,
    model_h,
    scale,
    pad_x,
    pad_y,
):
    """Decode Hailo's class-organized normalized NMS output."""
    image_h, image_w = image_shape[:2]
    detections = []

    for class_id, class_detections in enumerate(output):
        for det in class_detections:
            if len(det) < 5:
                continue

            ymin, xmin, ymax, xmax, score = det[:5]
            if score < CONF_THRESHOLD:
                continue

            x1 = (xmin * model_w - pad_x) / scale
            x2 = (xmax * model_w - pad_x) / scale
            y1 = (ymin * model_h - pad_y) / scale
            y2 = (ymax * model_h - pad_y) / scale

            x1 = int(np.clip(x1, 0, image_w - 1))
            x2 = int(np.clip(x2, 0, image_w - 1))
            y1 = int(np.clip(y1, 0, image_h - 1))
            y2 = int(np.clip(y2, 0, image_h - 1))

            detections.append(
                {
                    "class_id": class_id,
                    "score": float(score),
                    "bbox": (x1, y1, x2, y2),
                }
            )

    return detections


def run_object_detection(img, model: HailoYOLO):
    """Run object detection on image and return filtered detections (class 0 only).

    Args:
        img: numpy array from camera (BGR format)
        model: HailoYOLO instance

    Returns:
        List of detections with only class 0: [{"box": [...], "score": ...}]
    """
    # Convert BGR to RGB for inference
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # Run inference
    all_detections = model.infer(img_rgb)

    # Filter to keep only class 0
    class_0_detections = [
        {"bbox": det["bbox"], "score": det["score"]}
        for det in all_detections
        if det["class_id"] == 0
    ]

    return class_0_detections


def draw_detections_on_image(img, detections):
    """Draw detection boxes on image.

    Args:
        img: numpy array (BGR format) to draw on
        detections: list of detections with "bbox" and "score" keys

    Returns:
        Image with boxes drawn
    """
    for det in detections:
        x1, y1, x2, y2 = det["bbox"]
        score = det["score"]

        # Draw bounding box
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # Draw label
        label = f"human {score:.2f}"
        (w, h), _ = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
        )
        cv2.rectangle(
            img, (x1, max(y1 - h - 10, 0)), (x1 + w, y1), (0, 255, 0), -1
        )
        cv2.putText(
            img,
            label,
            (x1, y1 - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 0),
            1,
            cv2.LINE_AA,
        )

    return img


def init_camera():
    """Initialize RealSense camera and return pipeline + config."""
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, CAM_WIDTH, CAM_HEIGHT, rs.format.bgr8, CAM_FPS)
    pipeline.start(config)
    print(f"[CAMERA] Started color stream {CAM_WIDTH}x{CAM_HEIGHT}@{CAM_FPS}fps")
    return pipeline


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

    # Initialize camera
    pipeline = init_camera()

    # Initialize video writer for saving video.mp4
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video_writer = cv2.VideoWriter(
        VIDEO_MP4_PATH, fourcc, VIDEO_FPS, (CAM_WIDTH, CAM_HEIGHT)
    )
    print(f"[VIDEO] Recording to {VIDEO_MP4_PATH}")

    # Initialize Hailo YOLO model for object detection
    print(f"[DETECTION] Loading Hailo model: {HEF_PATH}")
    try:
        detection_model = HailoYOLO(HEF_PATH)
        print("[DETECTION] Model loaded successfully")
    except Exception as e:
        print(f"[DETECTION] Failed to load model: {e}")
        detection_model = None

    # FPS tracking variables
    frame_count = 0
    last_fps_time = time.monotonic()
    fps_display_interval = 1.0  # seconds
    current_fps = 0.0
    shoulder_pan_tracking_offset = 0.0

    # Oscillation parameters per joint: (amplitude_deg, freq_hz)
    JOINT_OSCILLATIONS = {
        "shoulder_pan":  (6.0,  1.2),
        "shoulder_lift": (4.0,  2.4),
        "elbow_flex":    (6.0,  2.4),  # opposite direction via negative amplitude
        "wrist_flex":    (3.0,  2.4),
        "wrist_roll":    (4.0,  1.2),
    }

    start_time = time.monotonic()

    # Forward-backward movement parameters (single 10 Hz cycle, 0.1s duration).
    DASH_amplitude_deg = {
        "shoulder_lift":  15.0,
        "elbow_flex":    -15.0,
        "wrist_flex":   -10.0,
    }
    DASH_FREQ_HZ = 2.0
    DASH_MIN_PERIOD = 3  # min time between DASH triggers
    DASH_MAX_PERIOD = 10  # max time between DASH triggers

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
    IMPACT_MIN_PERIOD = 3  # min time between impact triggers
    IMPACT_MAX_PERIOD = 10  # max time between impact triggers

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
            # --- Camera capture & FPS ---
            frames = pipeline.wait_for_frames()
            color = frames.get_color_frame()
            if color:
                img = np.asanyarray(color.get_data())

                # Run object detection if model is available
                if detection_model is not None:
                    try:
                        detections = run_object_detection(img, detection_model)
                        img = draw_detections_on_image(img, detections)
                        if detections and dash_in_progress==False and impact_in_progress==False:
                            highest_score_detection = max(
                                detections,
                                key=lambda detection: detection["score"],
                            )
                            x1, _, x2, _ = highest_score_detection["bbox"]
                            bbox_center_x = (x1 + x2) / 2
                            frame_center_x = img.shape[1] / 2

                            if bbox_center_x > frame_center_x:
                                shoulder_pan_tracking_offset += PAN_TRACKING_STEP_DEG
                            elif bbox_center_x < frame_center_x:
                                shoulder_pan_tracking_offset -= PAN_TRACKING_STEP_DEG
                    except Exception as e:
                        print(f"[DETECTION] Error: {e}")

                # Save every frame as cam.jpg (overwrites, so always latest)
                cv2.imwrite(CAM_JPEG_PATH, img)

                # Write frame to video.mp4
                video_writer.write(img)

                # FPS calculation
                frame_count += 1
                now = time.monotonic()
                if now - last_fps_time >= fps_display_interval:
                    current_fps = frame_count / (now - last_fps_time)
                    print(f"[CAMERA] FPS: {current_fps:.1f}", end="\r")
                    frame_count = 0
                    last_fps_time = now

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
                    #print(f"[{joint}] period {current_period} → amp={joint_random_amp[joint]:.2f}deg (base={amplitude_deg}deg)")

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
                    #print(f"[DASH] triggering forward-backward at t={elapsed:.2f}s (IN_PROGRESS)")

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
                    #print(f"[DASH] completed at t={elapsed:.2f}s (IN_PROGRESS=False)")

            # --- Heavy impact on top of oscillations + DASH ---
            if next_impact_time is None:
                # Schedule next impact 1-5 seconds from now.
                next_impact_time = elapsed + random.uniform(IMPACT_MIN_PERIOD, IMPACT_MAX_PERIOD)

            if impact_cycle_start is None:
                # Check if it's time to start an impact cycle.
                if elapsed >= next_impact_time:
                    impact_cycle_start = elapsed
                    impact_in_progress = True
                    #print(f"[IMPACT] triggering heavy impact at t={elapsed:.2f}s (IN_PROGRESS)")

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

                    #print(f"[IMPACT] t={impact_elapsed:.3f}s phase={'impact' if impact_elapsed < impact_duration else 'recovery'} sine={impact_sine:+.3f}")
                else:
                    # Impact cycle finished — reset for next random trigger.
                    impact_cycle_start = None
                    next_impact_time = None
                    impact_in_progress = False
                    #print(f"[IMPACT] completed at t={elapsed:.2f}s (IN_PROGRESS=False)")

            action["shoulder_pan.pos"] = round(
                action["shoulder_pan.pos"] + shoulder_pan_tracking_offset,
                2,
            )
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
        print("\n[CAMERA] Stopping camera stream.")
        try:
            pipeline.stop()
        except Exception:
            pass

        # Clean up detection model
        if detection_model is not None:
            try:
                detection_model.close()
            except Exception:
                pass

        # Release video writer
        try:
            video_writer.release()
            print(f"[VIDEO] Saved {VIDEO_MP4_PATH}")
        except Exception:
            pass

        print("Robot disconnected.")


if __name__ == "__main__":
    main()
