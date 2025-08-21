# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseStamped

# def run():
#     rclpy.init()
#     node = Node("pose_stamped_example")
#     pose = PoseStamped()
#     pose.pose.position.x = 1.0
#     pose.pose.position.y = 2.0
#     pose.pose.position.z = 3.0
#     node.get_logger().info(f"X: {pose.pose.position.x}")
#     node.get_logger().info(f"Y: {pose.pose.position.y}")
#     node.get_logger().info(f"Z: {pose.pose.position.z}")
#     node.destroy_node()
#     rclpy.shutdown()



#################################################################333
#!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseStamped

# def run():
#     rclpy.init()
#     node = Node("pose_stamped_example")
#     pose = PoseStamped()
#     pose.pose.position.x = 1.0
#     pose.pose.position.y = 2.0
#     pose.pose.position.z = 3.0
#     node.get_logger().info(f"X: {pose.pose.position.x}")
#     node.get_logger().info(f"Y: {pose.pose.position.y}")
#     node.get_logger().info(f"Z: {pose.pose.position.z}")
#     node.destroy_node()
#     rclpy.shutdown()



#################################################################333
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import numpy as np
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose

import rclpy
from rclpy.duration import Duration
from rclpy.time import Time

from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

class DepthTo3DNode(Node):
    def __init__(self):
        super().__init__('depth_to_3d')
        self.bridge = CvBridge()
        self.latest_depth = None
        self.camera_info = None

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.depth_sub = self.create_subscription(
            Image, "/intel_realsense_r200_depth/depth/image_raw",
            self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/intel_realsense_r200_depth/camera_info",
            self.camera_info_callback, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/object_in_odom', 10)
    
    def camera_info_callback(self, msg):
        self.camera_info = msg
        self.img_frame = "realsense_depth_frame"
        self.get_logger().info(f"Camera info received, using frame: {self.img_frame}")

    def depth_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.latest_depth = cv_image.astype(np.float32)
            self.process_depth()
        except Exception as e:
            self.get_logger().error(f"Depth conversion failed: {str(e)}")

    def process_depth(self):
        if self.latest_depth is None or self.camera_info is None:
            return

        h, w = self.latest_depth.shape[:2]
        x_center = w // 2
        y_center = h // 2

        depth = self.latest_depth[y_center, x_center]
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        X = (x_center - cx) * depth / fx
        Y = (y_center - cy) * depth / fy
        Z = float(depth)

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.img_frame
        pose.pose.position.x = X
        pose.pose.position.y = Y
        pose.pose.position.z = Z
        pose.pose.orientation.w = 1.0



        try:
            transform = self.tf_buffer.lookup_transform(
                "odom",
                self.img_frame,
                rclpy.time.Time()
            )
            #self.get_logger().info(f"odom_frame: {transform.header.frame_id}")
           # self.get_logger().info(f"camera_frame: {transform.child_frame_id}")
            t = transform.transform.translation
           # self.get_logger().info(f"relative coordinates: x={t.x}, y={t.y}, z={t.z}")
            self.get_logger().info("Transformed to 'odom' frame")

            ob2odom = self.tf_buffer.transform(pose, "odom", timeout=Duration(seconds=0.5))
            self.get_logger().info(
                f"object coordinates in odom frame: x={ob2odom.pose.position.x}, "
                f"y={ob2odom.pose.position.y}, z={ob2odom.pose.position.z}"
            )
            ob2odom.header.stamp = self.get_clock().now().to_msg()  # make sure stamp is valid
            ob2odom.header.frame_id = "odom"
            self.pose_pub.publish(ob2odom)
          

        except Exception as e:
            self.get_logger().error(f"TF lookup failed: {e}")



rclpy.init()
node = DepthTo3DNode()
try:
    rclpy.spin(node)
finally:
    node.destroy_node()
    rclpy.shutdown()
