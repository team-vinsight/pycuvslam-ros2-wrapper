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

1.  **Install Dependencies**:
    ```bash
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    pip install "numpy<2" opencv-python
    ```
    *Note: `numpy<2` is currently required due to compatibility issues with `cv_bridge`.*

2.  **Build the Package**:
    ```bash
    colcon build --packages-select pycuvslam_ros2
    source install/setup.bash
    ```

## Orbbec Gemini 2L Setup

This package includes a dedicated launch file for the **Orbbec Gemini 2L** camera that handles hardware synchronization and depth-to-color registration automatically.

1.  **Calibrate Camera**:
    *   You **must** provide precise intrinsic calibration parameters for your specific camera unit.
    *   Edit the file: `src/pycuvslam_ros2/config/gemini2l_params.yaml`.
    *   Update `fx`, `fy`, `cx`, `cy` with your values.

2.  **Launch System**:
    This command launches the camera driver, establishes hardware alignment, and starts the SLAM node with RViz.

    ```bash
    ros2 launch pycuvslam_ros2 pycuvslam_gemini2l_launch.py
    ```

## Generic Configuration (Other Cameras)

### Camera Parameters
You **must** provide precise intrinsic calibration parameters. 
Edit `config/camera_params.yaml` (or create your own).

Example structure:
```yaml
camera:
  width: 640
  height: 480
  fx: 517.3
  fy: 516.5
  cx: 318.6
  cy: 255.3
  distortion_model: 2 # 0=Pinhole, 2=Brown
  distortion_coeffs: [...]
```

### Launch Parameters
Key parameters in `launch/rgbd_launch.py`:
- `sys_path_pycuvslam`: Absolute path to the folder containing the `cuvslam` Python package.
- `rgb_topic`: ROS topic for RGB image.
- `depth_topic`: ROS topic for Depth image.
- `depth_scale_factor`: set to `1000.0` for mm-based depth (Orbbec/Realsense), or `5000.0` for TUM datasets.

## Testing with TUM RGB-D Dataset
A helper script `play_tum.py` is included to stream TUM dataset files.

1.  **Launch the SLAM Node**:
    ```bash
    ros2 launch pycuvslam_ros2 rgbd_launch.py depth_scale_factor:=5000.0
    ```

2.  **Run the Dataset Player**:
    ```bash
    # Usage: python3 src/pycuvslam_ros2/play_tum.py <path_to_extracted_folder>
    python3 src/pycuvslam_ros2/play_tum.py ~/Downloads/rgbd_dataset_freiburg3_long_office_household
    ```

## Topics

| Name | Type | Description |
|------|------|-------------|
| `/camera/rgb/image_raw` | `sensor_msgs/Image` | Subscribed RGB Input |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | Subscribed Depth Input |
| `/odom` | `nav_msgs/Odometry` | Published Odometry (Pose) |
| `/tf` | `tf2_msgs/TFMessage` | Published transform (`odom` -> `camera_link`) |

## Troubleshooting

- **`AttributeError: _ARRAY_API not found` in `cv_bridge`**:
  This means you have NumPy 2.0+ installed which breaks ROS 2 Humble's `cv_bridge`.
  **Fix**: `pip install "numpy<2"` or rely on the updated code in this package which manually handles conversion.

- **No motion in RViz (Identity Pose)**:
  - Check `depth_scale_factor`.
  - **Critical**: Check your config yaml allows correct intrinsics. Use the `gemini2l_params.yaml` for Gemini 2L and ensure `fx/fy` are correct.
  - Ensure `depth_registration` (hardware alignment) is enabled on the camera so depth matches RGB.
