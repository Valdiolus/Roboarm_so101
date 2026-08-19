import pyrealsense2 as rs
import numpy as np
import cv2
import os
from datetime import datetime

# Create output directory if it doesn't exist
output_dir = "recordings"
os.makedirs(output_dir, exist_ok=True)

fps = 15
width, height = 640, 480

# Generate filename with timestamp
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
output_path = os.path.join(output_dir, f"recording_{timestamp}.mp4")

# Configure pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)

profile = pipeline.start(config)

# Get codec and FPS for VideoWriter
fourcc = cv2.VideoWriter_fourcc(*'mp4v')


# Create VideoWriter
out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

# Create images subdirectory
images_dir = os.path.join(output_dir, "frames")
os.makedirs(images_dir, exist_ok=True)

# Create depth images subdirectory
depth_images_dir = os.path.join(output_dir, "depth_frames")
os.makedirs(depth_images_dir, exist_ok=True)

print(f"Recording to: {output_path}")
print("Press Ctrl+C to stop recording.")

try:
    frame_count = 0
    last_saved = -1
    while True:
        frames = pipeline.wait_for_frames()
        color = frames.get_color_frame()
        depth = frames.get_depth_frame()
        if not color:
            continue
        img = np.asanyarray(color.get_data())
        out.write(img)
        frame_count += 1
        # Save a color frame as image every 1 second (fps frames)
        saved_at = frame_count // fps
        if saved_at > last_saved:
            # Save color frame
            img_path = os.path.join(images_dir, f"frame_{timestamp}_{saved_at:04d}.png")
            cv2.imwrite(img_path, img)
            print(f"Saved color frame {frame_count} to: {img_path}")
            # Save depth frame
            if depth:
                depth_img = np.asanyarray(depth.get_data())
                # Normalize depth for visualization (16-bit to 8-bit)
                depth_img_normalized = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8UC1)
                depth_img_path = os.path.join(depth_images_dir, f"depth_{timestamp}_{saved_at:04d}.png")
                cv2.imwrite(depth_img_path, depth_img_normalized)
                print(f"Saved depth frame {frame_count} to: {depth_img_path}")
            last_saved = saved_at
        # Print progress every X frames (10 seconds)
        if frame_count % (fps*10) == 0:
            print(f"Recorded: {frame_count} frames")
finally:
    pipeline.stop()
    out.release()
    cv2.destroyAllWindows()
    print(f"Recording saved to: {output_path}")
    print(f"Total frames: {int(out.get(cv2.CAP_PROP_FRAME_COUNT))}")
    print(f"Saved frames to: {images_dir}")
    print(f"Saved depth frames to: {depth_images_dir}")
