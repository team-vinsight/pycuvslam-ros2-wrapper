#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import sys
import numpy as np
import time

class TumPlayer(Node):
    def __init__(self, dataset_path):
        super().__init__('tum_player')
        self.dataset_path = dataset_path
        
        self.rgb_pub = self.create_publisher(Image, '/camera/rgb/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        
        self.bridge = CvBridge()
        
        self.associations = self.associate_data(
            os.path.join(dataset_path, 'rgb.txt'),
            os.path.join(dataset_path, 'depth.txt')
        )
        
        print(f"Found {len(self.associations)} associated frames.")
        
    def read_file_list(self, filename):
        file_list = {}
        with open(filename) as f:
            for line in f:
                if line.startswith('#'): continue
                parts = line.strip().split()
                if len(parts) >= 2:
                    timestamp = float(parts[0])
                    filename = parts[1]
                    file_list[timestamp] = filename
        return file_list

    def associate_data(self, rgb_file, depth_file):
        rgb_list = self.read_file_list(rgb_file)
        depth_list = self.read_file_list(depth_file)
        
        matches = []
        rgb_keys = sorted(rgb_list.keys())
        depth_keys = sorted(depth_list.keys())
        
        offset = 0.0
        max_diff = 0.02
        
        depth_idx = 0
        for rgb_time in rgb_keys:
            best_dist = max_diff
            best_idx = -1
            
            # Simple association (assuming sorted)
            while depth_idx < len(depth_keys) and depth_keys[depth_idx] < rgb_time - max_diff:
                depth_idx += 1
                
            temp_idx = depth_idx
            while temp_idx < len(depth_keys) and depth_keys[temp_idx] <= rgb_time + max_diff:
                dist = abs(depth_keys[temp_idx] - rgb_time)
                if dist < best_dist:
                    best_dist = dist
                    best_idx = temp_idx
                temp_idx += 1
                
            if best_idx != -1:
                matches.append((rgb_time, rgb_list[rgb_time], depth_keys[best_idx], depth_list[depth_keys[best_idx]]))
        
        return matches

    def run(self):
        print("Starting playback. Press CSV+C to stop.")
        for i, (rgb_stamp, rgb_file, depth_stamp, depth_file) in enumerate(self.associations):
            rgb_path = os.path.join(self.dataset_path, rgb_file)
            depth_path = os.path.join(self.dataset_path, depth_file)
            
            rgb_img = cv2.imread(rgb_path, cv2.IMREAD_COLOR)
            depth_img = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED) # usually 16-bit PNG
            
            if rgb_img is None or depth_img is None:
                print(f"Failed to load {rgb_path} or {depth_path}")
                continue
                
            # Publish RGB
            rgb_msg = self.bridge.cv2_to_imgmsg(rgb_img, "bgr8")
            rgb_msg.header.stamp = self.get_clock().now().to_msg()
            rgb_msg.header.frame_id = "camera_link"
            self.rgb_pub.publish(rgb_msg)
            
            # Publish Depth
            # TUM depth is 16-bit mm (5000 scale factor usually). ROS expects 16UC1 (mm) or 32FC1 (m).
            # cv_bridge handles standard opencv types.
            depth_msg = self.bridge.cv2_to_imgmsg(depth_img, "16UC1")
            depth_msg.header = rgb_msg.header # Sync timestamps
            self.depth_pub.publish(depth_msg)
            
            print(f"Published frame {i+1}/{len(self.associations)}")
            
            # Simple rate control (30fps approx)
            time.sleep(0.033)

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 play_tum.py <path_to_dataset_folder>")
        sys.exit(1)
        
    rclpy.init()
    player = TumPlayer(sys.argv[1])
    try:
        player.run()
    except KeyboardInterrupt:
        pass
    finally:
        player.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
