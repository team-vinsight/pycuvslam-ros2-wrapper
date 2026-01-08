# Plan for PyCuVSLAM ROS 2 Wrapper (RGB-D)

This package will wrap the `PyCuVSLAM` library into a ROS 2 node, specifically for RGB-D camera configurations, following the architecture of `ORB_SLAM3_ROS2`.

## 1. Dependencies and Environment
- **PyCuVSLAM**: Located at `/home/intellisense05/pycuvslam`. Using `sys.path.append` to import.
- **ROS 2 Packages**: `rclpy`, `sensor_msgs`, `cv_bridge`, `message_filters`, `tf2_ros`, `geometry_msgs`, `image_transport`.
- **Python Libraries**: `numpy`, `opencv-python` (cv2).

## 2. Architecture
The architecture will mirror `ORB_SLAM3_ROS2`:
1.  **Node Class**: `CuVSLAMRGBDNode` (inherits `rclpy.node.Node`).
2.  **Synchronization**: Uses `message_filters` (ApproximateTime) to sync RGB and Depth image topics.
3.  **Core Logic**:
    -   Convert ROS images to OpenCV/Numpy.
    -   Process with `cuvslam.Tracker`.
    -   Publish results (Odometry/TF).

## 3. Implementation Details

### File Structure
```
src/pycuvslam_ros2/
├── package.xml             # Dependencies
├── setup.py                # Installation
├── launch/
│   └── rgbd_launch.py      # Launch file
├── config/
│   └── camera_params.yaml  # Camera intrinsics
└── pycuvslam_ros2/
    ├── __init__.py
    ├── node_rgbd.py        # Main Node Implementation
    └── cuvslam_wrapper.py  # (Optional) Helper to handle CuVSLAM specifics
```

### Components

#### A. Node Initialization (`node_rgbd.py`)
- **Parameters**: 
    - `sys_path_pycuvslam`: Path to pycuvslam installation.
    - `config_file`: Path to camera calibration YAML.
    - `rgb_topic`: Topic for RGB image.
    - `depth_topic`: Topic for Depth image.
    - `publish_tf`: Boolean to enable/disable TF broadcasting.
    - `base_frame_id`: Frame ID for the robot/camera.
    - `odom_frame_id`: Frame ID for the fixed world/odometry frame.
- **Setup**:
    - Parse YAML config to create `cuvslam.Camera` and `cuvslam.Rig`.
    - Initialize `cuvslam.Tracker` with `OdometryMode.RGBD`.
    - Setup Subscribers:
        ```python
        rgb_sub = message_filters.Subscriber(self, Image, rgb_topic)
        depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], ...)
        ts.registerCallback(self.rgbd_callback)
        ```
    - Setup Publishers:
        - `nav_msgs/Odometry` for pose.
        - `tf2_ros.TransformBroadcaster`.

#### B. Callback Loop (`rgbd_callback`)
1.  Convert ROS `Image` -> Numpy array (using `cv_bridge`).
    -   RGB: `bgr8` or `rgb8` -> Ensure correct format for CuVSLAM.
    -   Depth: `16UC1` or `32FC1` -> Convert to what CuVSLAM expects (likely float meters or uint16 mm, need to check/parameterize scale).
2.  Call `tracker.track(timestamp, images=[rgb], depths=[depth])`.
3.  Process Result:
    -   Get `pose.world_from_rig`.
    -   Publish TF (`odom` -> `camera_link`).
    -   Publish Odometry message.

#### C. Configuration
Camera parameters need to be loaded into `cuvslam.Camera` object:
- `width`, `height`
- `focal` (fx, fy)
- `principal` (cx, cy)
- `distortion` (model + coeffs)

## 4. Todo List
1.  [ ] Update `package.xml` and `setup.py`.
2.  [ ] Create `config/ex_camera.yaml` template.
3.  [ ] Create `launch/rgbd_launch.py`.
4.  [ ] Implement `pycuvslam_ros2/node_rgbd.py`.
