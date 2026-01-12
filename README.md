# PyCuVSLAM ROS 2 Wrapper with Orbbec ROS2 SDK

This workspace contains packages for running NVIDIA PyCuVSLAM with Orbbec RGB-D cameras on ROS 2 Humble.

## Packages

*   **`src/OrbbecSDK_ROS2`**: Official ROS 2 SDK and driver for Orbbec cameras (Gemini 2, Femto, Astra, etc.).
*   **`src/pycuvslam_ros2`**: A ROS 2 wrapper for NVIDIA's [PyCuVSLAM](https://github.com/NVlabs/PyCuVSLAM) library.

## Build Instructions

1.  **Install Dependencies**:
    ```bash
    # Install ROS dependencies
    rosdep install --from-paths src --ignore-src -r -y

    # Install Python dependencies (NumPy < 2 is required for cv_bridge compatibility)
    pip install "numpy<2" opencv-python
    ```

2.  **Build Workspace**:
    ```bash
    colcon build --symlink-install
    ```

3.  **Source Environment**:
    ```bash
    source install/setup.bash
    ```

## Documentation

*   For **Setup, Configuration, and Usage** of the SLAM system, see the [PyCuVSLAM ROS 2 README](src/pycuvslam_ros2/README.md).
