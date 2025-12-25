from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig
from lerobot.cameras.realsense.camera_realsense import RealSenseCamera
from lerobot.cameras.configs import ColorMode, Cv2Rotation
from lerobot.teleoperators.so101_leader import SO101LeaderConfig, SO101Leader
from lerobot.robots.so101_follower import SO101FollowerConfig, SO101Follower
import time
import cv2

#60 fps max
rs2_config = RealSenseCameraConfig(
    serial_number_or_name="825312071606",
    fps=60,
    width=640,
    height=480,
    color_mode=ColorMode.RGB,
    use_depth=True,
    rotation=Cv2Rotation.NO_ROTATION
)

#camera = RealSenseCamera(rs2_config)
#camera.connect()
camera_configs = {
    "front": rs2_config,   # key name can be anything, but be consistent
}

robot_config = SO101FollowerConfig(
    port="/dev/ttyACM0",
    id="valdis_follower_arm",
    cameras=camera_configs
)

teleop_config = SO101LeaderConfig(
    port="/dev/ttyACM1",
    id="valdis_leader_arm",
)

robot = SO101Follower(robot_config)
teleop_device = SO101Leader(teleop_config)
robot.connect()
teleop_device.connect()

loop_times = []

while True:
    loop_start = time.time()

    observation = robot.get_observation()
    
    action = teleop_device.get_action()
    robot.send_action(action)

    #show stream from camera
    img = observation['front']
    #bgr to rgb conversion
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    cv2.imshow("RealSense Color", img)
    if cv2.waitKey(1) & 0xFF == 27:  # ESC
        break
    
    loop_time = time.time() - loop_start
    loop_times.append(loop_time)
    
    # Update statistics every 100 iterations
    if len(loop_times) >= 100:
        avg_time = sum(loop_times) / len(loop_times)
        min_time = min(loop_times)
        max_time = max(loop_times)
        print(f"\rLoop time - Avg: {avg_time*1000:.2f}ms, Min: {min_time*1000:.2f}ms, Max: {max_time*1000:.2f}ms, Freq: {1/avg_time:.1f}Hz, camera {observation['front'].shape}", end='', flush=True)
        loop_times = []