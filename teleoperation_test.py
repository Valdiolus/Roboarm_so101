import time
from lerobot.teleoperators.so_leader import SO101LeaderConfig, SO101Leader
from lerobot.robots.so_follower import SO101FollowerConfig, SO101Follower

robot_config = SO101FollowerConfig(
    port="/dev/tty.usbmodem5AAF2197991",
    id="valdis_new_gripper_follower_arm",
)

teleop_config = SO101LeaderConfig(
    port="/dev/tty.usbmodem5AAF2197991",
    id="valdis_old_leader_arm",
) 

robot = SO101Follower(robot_config)
teleop_device = SO101Leader(teleop_config)
robot.connect()
teleop_device.connect()

loop_times = []

while True:
    loop_start = time.time()
    
    action = teleop_device.get_action()
    robot.send_action(action)
    
    loop_time = time.time() - loop_start
    loop_times.append(loop_time)
    
    # Update statistics every 100 iterations
    if len(loop_times) >= 1000:
        avg_time = sum(loop_times) / len(loop_times)
        min_time = min(loop_times)
        max_time = max(loop_times)
        print(f"\rLoop time - Avg: {avg_time*1000:.2f}ms, Min: {min_time*1000:.2f}ms, Max: {max_time*1000:.2f}ms, Freq: {1/avg_time:.1f}Hz", end='', flush=True)
        # print action values
        print(f"\rAction: {action}", end='', flush=True)
        loop_times = []