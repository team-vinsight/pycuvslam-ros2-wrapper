# PyCuVSLAM ROS 2 Wrapper

This package provides a ROS 2 wrapper for NVIDIA's [PyCuVSLAM](https://github.com/NVlabs/PyCuVSLAM), a CUDA-accelerated Visual SLAM library. This wrapper is specifically designed for **RGB-D** camera configurations.

## Prerequisites

### Hardware
*   **GPU**: NVIDIA GPU with CUDA support.
*   **Driver**: NVIDIA Driver **555 or higher** (Required for CUDA 12.6 support).
    *   *Verified with Driver Version 580.105.08*.

### Software
*   **OS**: Ubuntu 22.04 (Jammy Jellyfish).
*   **ROS 2**: Humble Hawksbill (or compatible).
*   **Python**: 3.10+.
*   **PyCuVSLAM**: The library must be installed or available on your system.

## Installation

1.  **Clone the repository** (if not already done):
    ```bash
    cd ~/ros2_ws/src
    # Clone or place this package here
    ```

2.  **Install Dependencies**:
    ```bash
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    pip install "numpy<2" opencv-python
    ```
    *Note: `numpy<2` is currently required due to compatibility issues with `cv_bridge`.*

3.  **Build the Package**:
    ```bash
    colcon build --packages-select pycuvslam_ros2
    source install/setup.bash
    ```

## Configuration

### Camera Parameters
You **must** provide precise intrinsic calibration parameters for your camera. 
Edit the file at `config/camera_params.yaml` (or the installed version in `install/pycuvslam_ros2/share/...`).

Example structure (`config/camera_params.yaml`):
```yaml
camera:
  width: 640
  height: 480
  fx: 517.3
  fy: 516.5
  cx: 318.6
  cy: 255.3
  # Distortion model: 0=Pinhole, 1=Fisheye, 2=Brown, 3=Polynomial
  distortion_model: 2
  # For Brown: [k1, k2, p1, p2, k3]
  distortion_coeffs: [0.2624, -0.9531, -0.0054, 0.0026, 1.1633]
```

### Launch Parameters
Key parameters in `launch/rgbd_launch.py`:
- `sys_path_pycuvslam`: Absolute path to the folder containing the `cuvslam` Python package (e.g., `.../bin/x86_64`).
- `rgb_topic`: ROS topic for RGB image (default: `/camera/rgb/image_raw`).
- `depth_topic`: ROS topic for Depth image (default: `/camera/depth/image_raw`).
- `depth_scale_factor`: Scale factor to convert depth values. 
    - For TUM Datasets (uint16 where 5000 = 1m), set to `5000.0`.
    - For metric float images, set to `1.0`.

## Usage

### 1. Running the Node
This launch file starts the SLAM node and RViz2 for visualization.

```bash
source install/setup.bash
ros2 launch pycuvslam_ros2 rgbd_launch.py
```

### 2. Testing with TUM RGB-D Dataset
A helper script `play_tum.py` is included to easily stream TUM dataset files to the ROS topics.

1.  **Download a Dataset** (e.g., freiburg1_xyz):
    ```bash
    mkdir -p ~/Downloads/tum_rgbd
    cd ~/Downloads/tum_rgbd
    wget https://cvg.cit.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_xyz.tgz
    tar -xzf rgbd_dataset_freiburg1_xyz.tgz
    ```

2.  **Launch the SLAM Node** (Terminal 1):
    Ensure `depth_scale_factor` is set for TUM (5000.0).
    ```bash
    ros2 launch pycuvslam_ros2 rgbd_launch.py depth_scale_factor:=5000.0
    ```

3.  **Run the Dataset Player** (Terminal 2):
    ```bash
    source install/setup.bash
    # Usage: python3 src/pycuvslam_ros2/play_tum.py <path_to_extracted_folder>
    python3 src/pycuvslam_ros2/play_tum.py ~/Downloads/tum_rgbd/rgbd_dataset_freiburg1_xyz
    ```

## Topics

| Name | Type | Description |
|------|------|-------------|
| `/camera/rgb/image_raw` | `sensor_msgs/Image` | Subscribed RGB Input |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | Subscribed Depth Input |
| `/odom` | `nav_msgs/Odometry` | Published Odometry (Pose) |
| `/tf` | `tf2_msgs/TFMessage` | Published transform (`odom` -> `camera_link`) |

## Troubleshooting

- **`ImportError: No module named 'cuvslam'`**: 
  Ensure `sys_path_pycuvslam` points to the directory containing the `cuvslam` package (often `bin/x86_64` inside the repo).

- **`Module compiled using NumPy 1.x cannot be run in NumPy 2.x`**:
  Downgrade numpy: `pip install "numpy<2"`.

- **`Failed to initialize Tracker: ... requires CUDA 12.6`**:
  Update your NVIDIA drivers to version 555 or newer.

- **No motion in RViz**:
  - Check if `depth_scale_factor` is correct.
  - Check `camera_params.yaml` matches your camera. Incorrect calibration leads to tracking failure (identity pose).
