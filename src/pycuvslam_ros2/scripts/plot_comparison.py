#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
import sys
import os

def load_traj(filename):
    data = []
    if not os.path.exists(filename):
        print(f"File {filename} not found.")
        return np.array([])
        
    with open(filename, 'r') as f:
        for line in f:
            if line.startswith('#'): continue
            parts = line.strip().split()
            if len(parts) >= 4:
                # time x y z ...
                data.append([float(x) for x in parts[:4]])
    return np.array(data)

def main():
    file1 = 'standalone_traj.txt'
    file2 = 'ros_traj.txt'
    
    print(f"Loading {file1}...")
    traj1 = load_traj(file1)
    
    print(f"Loading {file2}...")
    traj2 = load_traj(file2)
    
    if len(traj1) == 0 and len(traj2) == 0:
        print("No data found.")
        return

    plt.figure(figsize=(10, 6))
    
    if len(traj1) > 0:
        # Align start to 0,0 for visual comparison if needed, or just plot raw
        # Usually SLAM starts at 0,0,0.
        plt.plot(traj1[:, 1], traj1[:, 2], label='Standalone (Python)', linewidth=2, alpha=0.7)
        
    if len(traj2) > 0:
        plt.plot(traj2[:, 1], traj2[:, 2], '--', label='ROS Wrapper', linewidth=2, alpha=0.7)
        
    plt.title("Trajectory Comparison (XY Plane)")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    
    output_img = 'trajectory_comparison.png'
    plt.savefig(output_img)
    print(f"Saved plot to {output_img}")

if __name__ == '__main__':
    main()
