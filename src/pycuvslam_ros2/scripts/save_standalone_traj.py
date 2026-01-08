#!/usr/bin/env python3
import sys
import os
import yaml
import numpy as np
import cv2

# Add pycuvslam bin directory to path
sys.path.append('/home/intellisense05/pycuvslam/bin/x86_64')
import cuvslam

# Add pycuvslam example directory to path to import dataset_utils
example_dir = '/home/intellisense05/pycuvslam/examples/tum'
sys.path.append(example_dir)
from dataset_utils import get_matched_rgbd_pairs

def load_frame(path):
    img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    if img is None: return None
    if len(img.shape) == 3: img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    return np.ascontiguousarray(img)

def main():
    # Set up dataset path
    # Using the path we established earlier
    tum_dataset_path = '/home/intellisense05/pycuvslam/examples/tum/dataset/rgbd_dataset_freiburg3_long_office_household'
    config_file_path = '/home/intellisense05/pycuvslam/examples/tum/freiburg3_rig.yaml'

    print(f"Processing dataset from: {tum_dataset_path}")
    
    # Get matched RGB-D pairs from dataset
    # Note: Using same params as track_tum.py
    rgbd_pairs = get_matched_rgbd_pairs(
        tum_dataset_path, max_time_diff=0.02, max_gap=0.5
    )
    print(f"Found {len(rgbd_pairs)} matched RGB-D pairs")

    # Load tum configuration
    with open(config_file_path, 'r') as file:
        config_data = yaml.safe_load(file)

    # Set up camera parameters
    camera = cuvslam.Camera()
    camera.size = (
        config_data['rgb_camera']['image_width'],
        config_data['rgb_camera']['image_height']
    )
    camera.principal = config_data['rgb_camera']['principal_point']
    camera.focal = config_data['rgb_camera']['focal_length']
    # These borders were in track_tum.py, keeping them for consistency
    camera.border_top = 20
    camera.border_bottom = 20
    camera.border_left = 10
    camera.border_right = 50

    # Set up RGBD settings
    rgbd_settings = cuvslam.Tracker.OdometryRGBDSettings()
    rgbd_settings.depth_scale_factor = config_data['depth_camera']['scale']
    rgbd_settings.depth_camera_id = 0
    rgbd_settings.enable_depth_stereo_tracking = False

    # Configure tracker
    cfg = cuvslam.Tracker.OdometryConfig(
        async_sba=True,
        enable_final_landmarks_export=False, # Don't need landmarks for this
        odometry_mode=cuvslam.Tracker.OdometryMode.RGBD,
        rgbd_settings=rgbd_settings
    )

    # Initialize tracker
    tracker = cuvslam.Tracker(cuvslam.Rig([camera]), cfg)
    cuvslam.warm_up_gpu()

    output_file = 'standalone_traj.txt'
    print(f"Running Tracking and saving to {output_file}...")
    
    with open(output_file, 'w') as f:
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        
        frame_id = 0
        for rgb_time, rgb_path, depth_path in rgbd_pairs:
            color_frame = load_frame(rgb_path)
            depth_frame = load_frame(depth_path)
            
            if color_frame is None or depth_frame is None:
                continue
            
            # Timestamp in ns
            timestamp = int(rgb_time * 1e9)
            
            # Track
            odom_pose_estimate, _ = tracker.track(
                timestamp, images=[color_frame], depths=[depth_frame]
            )
            
            if odom_pose_estimate.world_from_rig:
                pose = odom_pose_estimate.world_from_rig.pose
                t = pose.translation
                q = pose.rotation
                # Write to file
                f.write(f"{rgb_time} {t[0]} {t[1]} {t[2]} {q[0]} {q[1]} {q[2]} {q[3]}\n")
            
            frame_id += 1
            if frame_id % 100 == 0:
                print(f"Processed {frame_id} frames...")

    print("Done.")

if __name__ == '__main__':
    main()
