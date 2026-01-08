import sys
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import yaml
import os

from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import TransformStamped, PoseStamped
from cv_bridge import CvBridge
import message_filters
import tf2_ros

class CuVSLAMRGBDNode(Node):
    def __init__(self):
        super().__init__('cuvslam_rgbd_node')
        
        # Parameters
        self.declare_parameter('sys_path_pycuvslam', '/home/intellisense05/pycuvslam/bin/x86_64')
        self.declare_parameter('config_file', '')
        self.declare_parameter('rgb_topic', '/camera/rgb/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('base_frame_id', 'camera_link')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('depth_scale_factor', 1000.0) # Factor to divide by to get meters. If mm, 1000. If meters, 1.
        self.declare_parameter('use_gpu', True)

        sys_path = self.get_parameter('sys_path_pycuvslam').get_parameter_value().string_value
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        rgb_topic = self.get_parameter('rgb_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.base_frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value
        self.odom_frame_id = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        self.depth_scale = self.get_parameter('depth_scale_factor').get_parameter_value().double_value
        use_gpu = self.get_parameter('use_gpu').get_parameter_value().bool_value

        # Import PyCuVSLAM
        if sys_path not in sys.path:
            sys.path.append(sys_path)
            self.get_logger().info(f"Added {sys_path} to sys.path")
        
        try:
            import cuvslam
            # Import classes directly from cuvslam or submodules as needed
            # Based on docs: cuvslam.Tracker, cuvslam.Rig, etc.
            from cuvslam import Tracker, Rig, Camera
            # Odometry and Slam are under the core namespace
            CuvOdometry = cuvslam.core.Odometry
            Slam = cuvslam.core.Slam
            
            self.get_logger().info(f"Successfully imported cuvslam version: {cuvslam.get_version()}")
        except ImportError as e:
            self.get_logger().error(f"Failed to import pycuvslam from {sys_path}. Error: {e}")
            self.get_logger().error("Please ensure the path is correct and dependencies are installed.")
            # We can't continue without library
            return

        # Load Camera Config
        if not config_file or not os.path.exists(config_file):
            self.get_logger().error(f"Config file not provided or does not exist: {config_file}")
            return
            
        with open(config_file, 'r') as f:
            config_data = yaml.safe_load(f)
        
        cam_params = config_data.get('camera', {})
        
        # Initialize Camera
        self.get_logger().info("Initializing Camera...")
        camera = Camera()
        camera.size = (int(cam_params.get('width', 640)), int(cam_params.get('height', 480)))
        camera.focal = (float(cam_params.get('fx', 525.0)), float(cam_params.get('fy', 525.0)))
        camera.principal = (float(cam_params.get('cx', 320.0)), float(cam_params.get('cy', 240.0)))
        
        # Distortion
        dist_model = int(cam_params.get('distortion_model', 0)) # 0: Pinhole, 2: Brown
        camera.distortion.model = dist_model 
        
        coeffs = cam_params.get('distortion_coeffs', [])
        if coeffs:
             # camera.distortion.parameters might need to be set with a list of floats
             camera.distortion.parameters = [float(x) for x in coeffs]

        self.get_logger().info(f"Camera initialized: {camera.size}, {camera.focal}, Model: {camera.distortion.model}")

        # Initialize Rig
        rig = Rig()
        rig.cameras = [camera]
        
        # Initialize Tracker
        odom_config = CuvOdometry.Config()
        odom_config.odometry_mode = CuvOdometry.OdometryMode.RGBD
        odom_config.use_gpu = use_gpu
        
        # RGBD Settings
        odom_config.rgbd_settings.depth_camera_id = 0 
        # For TUM dataset (uint16 mm), scale factor is typically 5000.0 (5000 units = 1 meter)
        # However, default usually 1000.0. 
        # If PyCuVSLAM expects us to tell it the scale of the input image:
        # "depth_scale_factor: Scale factor for depth measurements".
        # If I pass uint16, does it divide by this factor to get meters?
        # If so, for TUM I should set this to 5000 if raw, OR I can pre-scale.
        # Let's trust the parameter.
        odom_config.rgbd_settings.depth_scale_factor = self.depth_scale

        self.get_logger().info("Initializing Tracker...")
        try:
            self.tracker = Tracker(rig, odom_config)
            cuvslam.warm_up_gpu()
            self.get_logger().info("Tracker Initialized and GPU warmed up.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize Tracker: {e}")
            return
        
        # ROS Setup
        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.path_pub = self.create_publisher(Path, 'path', 10)
        
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.odom_frame_id
        
        # Subscribers
        self.rgb_sub = message_filters.Subscriber(self, Image, rgb_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        
        # Sync
        queue_size = 10
        slop = 0.1 # seconds
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size, slop)
        self.ts.registerCallback(self.rgbd_callback)
        
        self.get_logger().info("Waiting for images...")

    def rgbd_callback(self, rgb_msg, depth_msg):
        try:
            # Convert images
            rgb_img = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            rgb_img = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2RGB)
            
            # Depth Handling
            # PyCuVSLAM expects 2D uint16 (mm) for depth images.
            if depth_msg.encoding == '16UC1':
                depth_img = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")
            elif depth_msg.encoding == '32FC1':
                # Convert float meters to uint16 mm
                depth_f = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
                depth_img = (depth_f * 1000.0).astype(np.uint16)
            else:
                self.get_logger().warn(f"Unsupported depth encoding: {depth_msg.encoding}")
                return

            # Ensure contiguous array (sometimes required by C++ bindings)
            rgb_img = np.ascontiguousarray(rgb_img)
            depth_img = np.ascontiguousarray(depth_img)

            # Metadata for track call
            # Need to verify if 'depths' argument expects [H,W] uint16 or something else.
            # Error said: "Depth must be 2D uint16 ndarray of shape (height, width)."
            # My current depth_img should be that if 16UC1.
            
            # Timestamp (ns)
            timestamp = rgb_msg.header.stamp.sec * 1_000_000_000 + rgb_msg.header.stamp.nanosec
            
            # Track
            # API: track(timestamp: int, images: List[Any], masks: List[Any] | None = None, depths: List[Any] | None = None)
            pose_est, slam_pose = self.tracker.track(
                timestamp=timestamp,
                images=[rgb_img], # List for 1 camera
                depths=[depth_img] # List for 1 camera
            )
            
            if pose_est:
                # Debug print for first few valid poses
                obj = pose_est.world_from_rig
                if hasattr(obj, 'pose'): p = obj.pose
                else: p = obj
                
                # Check if non-zero
                if np.sum(np.abs(p.translation)) > 0 or p.rotation[3] != 1.0:
                     self.get_logger().info(f"Valid Motion detected: T={p.translation}", throttle_duration_sec=1.0)
                else:
                     self.get_logger().info(f"Tracking Identity (Zero motion) detected.", throttle_duration_sec=1.0)

                self.publish_pose(pose_est, rgb_msg.header.stamp)
            else:
                 self.get_logger().warn(f"Tracking lost or not initialized. Timestamp: {timestamp}", throttle_duration_sec=1.0)

        except Exception as e:
            self.get_logger().error(f"Tracking error: {e}")

    def publish_pose(self, pose_est, timestamp_ros):
        # pose_est.world_from_rig (Pose object inside PoseWithCovariance potentially?)
        # Docs say: property world_from_rig: Rig pose in the world coordinate frame
        # We need to inspect what world_from_rig actually returns.
        # It seems it might be returning a PoseWithCovariance instead of just Pose, or the attribute nesting is different.
        
        # If pose_est.world_from_rig is a Pose:
        #   .translation (3-vec)
        #   .rotation (4-vec)
        # If it's PoseWithCovariance (based on API hint in __init__.py imports):
        #   .pose (the Pose)
        #   .covariance (6x6)
        
        # Let's handle both cases dynamically
        obj = pose_est.world_from_rig
        
        # Check if it wraps a pose
        if hasattr(obj, 'pose'):
            pose = obj.pose
        else:
            pose = obj

        # Now extract translation/rotation
        t = pose.translation # 3-vector
        q = pose.rotation # x, y, z, w
        
        # Publish TF
        t_stamped = TransformStamped()
        t_stamped.header.stamp = timestamp_ros
        t_stamped.header.frame_id = self.odom_frame_id
        t_stamped.child_frame_id = self.base_frame_id
        
        t_stamped.transform.translation.x = float(t[0])
        t_stamped.transform.translation.y = float(t[1])
        t_stamped.transform.translation.z = float(t[2])
        t_stamped.transform.rotation.x = float(q[0])
        t_stamped.transform.rotation.y = float(q[1])
        t_stamped.transform.rotation.z = float(q[2])
        t_stamped.transform.rotation.w = float(q[3])
        
        self.tf_broadcaster.sendTransform(t_stamped)
        
        # Publish Odometry
        odom_msg = Odometry()
        odom_msg.header = t_stamped.header
        odom_msg.child_frame_id = t_stamped.child_frame_id
        odom_msg.pose.pose.position.x = float(t[0])
        odom_msg.pose.pose.position.y = float(t[1])
        odom_msg.pose.pose.position.z = float(t[2])
        odom_msg.pose.pose.orientation = t_stamped.transform.rotation

        # Publish Path
        pose_stamped = PoseStamped()
        pose_stamped.header = t_stamped.header
        pose_stamped.pose.position = odom_msg.pose.pose.position
        pose_stamped.pose.orientation = odom_msg.pose.pose.orientation
        
        self.path_msg.header.stamp = t_stamped.header.stamp
        self.path_msg.poses.append(pose_stamped)
        self.path_pub.publish(self.path_msg)
        
        # TODO: Covariance? PyCuVSLAM might provide it but PoseEstimate seems just Pose.
        
        self.odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CuVSLAMRGBDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
