#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import torch
import torch.nn.functional as F
import torchvision
from torchvision import transforms
from PIL import Image as PILImage
import clip
import cv2
import json
import os
from datetime import datetime
import numpy as np
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo
import time as pytime




    

class ClipDetectionNode(Node):
    def __init__(self):
        super().__init__('clip_detector')
        self.bridge = CvBridge()
        self.device = "cuda" if torch.cuda.is_available() else "cpu"

        #camera init
        self.latest_depth = None
        self.camera_info = None
        self.latest_stamp = None
        self.latest_rgb = None
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10))
        self.tf_listener = TransformListener(self.tf_buffer, self)


        # CLIP + Faster R-CNN
        self.clip_model, self.clip_preprocess = clip.load("ViT-B/32", device=self.device)
        self.frcnn_model = torchvision.models.detection.fasterrcnn_resnet50_fpn_v2(weights="DEFAULT").eval()

        # Labels and score tracking
        self.labels = ["Refrigerator","water dispenser", "sofa", "white toilet",  "office chair with wheels"]
        
        # Initialize score tracking
        self.score_records_file = "object_detection_vlm.json"
        self.score_records = self.load_score_records()
        
        # Subscribers & Publishers
        self.pose_pub = self.create_publisher(PoseStamped, "/object_in_odom", 10)

        self.rgb_sub = self.create_subscription(
            Image,
            "/intel_realsense_r200_rgb/image_raw",
            self.rgb_callback,
            10
        )
        self.depth_sub = self.create_subscription(
            Image,
            "/intel_realsense_r200_depth/depth/image_raw",
            self.depth_callback,
            10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/intel_realsense_r200_depth/camera_info",
            self.camera_info_callback, 10)
        self.annotated_pub = self.create_publisher(Image, "/percep/annotated_image", 10)


    def camera_info_callback(self, msg):
        self.camera_info = msg
        self.img_frame = "realsense_depth_frame"
        self.get_logger().info(f"Camera info received, using frame: {self.img_frame}")

        

    def load_score_records(self):
        """Load existing score records or create new ones"""
        if os.path.exists(self.score_records_file):
            with open(self.score_records_file, 'r') as f:
                try:
                    return json.load(f)
                except json.JSONDecodeError:
                    self.get_logger().warn("Score file corrupted, creating new one")
        
        # Initialize with empty records
        return {label: {"highest_score": 0.0, "last_detected": None, "position":None} for label in self.labels}

    def save_score_records(self):
        """Save current score records to file"""
        with open(self.score_records_file, 'w') as f:
            json.dump(self.score_records, f, indent=4)

    def update_score_records(self, label, score, position=None):
        """Update records if this is the highest score for this label"""
        current_time = datetime.now().isoformat()
        
        if label not in self.score_records:
            self.score_records[label] = {
                "highest_score": score,
                "last_detected": current_time,
                "position": position
            }
            self.save_score_records()
            return True
        
        updated = False
        if score > self.score_records[label]["highest_score"]:
            self.score_records[label]["highest_score"] = score
            updated = True
        
        self.score_records[label]["last_detected"] = current_time
        if position is not None:
            self.score_records[label]["position"] = position
        if updated:
            self.save_score_records()
        return updated


    def rgb_callback(self, msg):
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_stamp = msg.header.stamp  # <-- Correct assignment
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge error: {e}")
            return
        self.process_frame()


    def depth_callback(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except CvBridgeError as e:
            self.get_logger().error(f"Depth CvBridge error: {e}")
            return
        self.process_frame()

    def process_frame(self):
        if self.latest_rgb is None or self.latest_depth is None:
            return

        cv_img = self.latest_rgb.copy()
        image_tensor = transforms.ToTensor()(cv_img)

        with torch.no_grad():
            frcnn_output = self.frcnn_model([image_tensor])[0]
        candidate_boxes = frcnn_output['boxes']

        best_labels_per_box = {}  # key: box index, value: (label, score)

        for i, box in enumerate(candidate_boxes):
            max_score = -1
            best_label = None
            for label in self.labels:
                _, score = self.match_label_box(label, image_tensor, [box])
                if score > max_score:
                    max_score = score
                    best_label = label

            if max_score >= 0.25 and best_label is not None:
                best_labels_per_box[i] = (best_label, max_score)
                # Initialize is_new_record as False first
                is_new_record = False
                position = None

                if self.latest_depth is not None and self.camera_info is not None:
                    box_index = i
                    box = candidate_boxes[box_index]
                    x_min, y_min, x_max, y_max = map(int, box)
                    x_center = (x_min + x_max) // 2
                    y_center = (y_min + y_max) // 2
                    depth = float(self.latest_depth[y_center, x_center])
                    if np.isnan(depth) or depth <= 0.0:
                        continue

                    fx = self.camera_info.k[0]
                    fy = self.camera_info.k[4]
                    cx = self.camera_info.k[2]
                    cy = self.camera_info.k[5]

                    X = (x_center - cx) * depth / fx
                    Y = (y_center - cy) * depth / fy
                    Z = depth

                    pose = PoseStamped()
                    pose.header.stamp = Time().to_msg()  
                    pose.header.frame_id = self.img_frame
                    pose.pose.position.x = X
                    pose.pose.position.y = Y
                    pose.pose.position.z = Z
                    pose.pose.orientation.w = 1.0


                    timeout_sec = 1.0
                    start_time = pytime.time()
                    transform = None
                    while pytime.time() - start_time < timeout_sec:
                        try:
                            transform = self.tf_buffer.lookup_transform(
                                "odom",
                                self.img_frame,
                                rclpy.time.Time()
                            )
                        # self.get_logger().info(f"odom_frame: {transform.header.frame_id}")
                        # self.get_logger().info(f"camera_frame: {transform.child_frame_id}")
                            t = transform.transform.translation
                            self.get_logger().info("Transformed to 'odom' frame")

                            ob2odom = self.tf_buffer.transform(pose, "odom", timeout=Duration(seconds=0.5))

                            self.pose_pub.publish(ob2odom)
                            self.get_logger().info(
                            f"object coordinates in odom frame: x={ob2odom.pose.position.x}, "
                            f"y={ob2odom.pose.position.y}, z={ob2odom.pose.position.z}"
                            )

                            position = {
                                "x": ob2odom.pose.position.x,
                                "y": ob2odom.pose.position.y,
                                "z": ob2odom.pose.position.z
                            }
                            # Update JSON with position
                            is_new_record = self.update_score_records(best_label, max_score, position=position)
                            ob2odom.header.stamp = self.get_clock().now().to_msg()  # make sure stamp is valid
                            ob2odom.header.frame_id = "odom"
                        except Exception as e:
                            self.get_logger().error(f"TF lookup failed: {e}")
                            is_new_record = False
                            pytime.sleep(0.1)  # Add this line
                    if transform is None:
                        self.get_logger().error("TF lookup failed after waiting.")
                        return


                # Logging
                log_msg = f"{best_label} detected with confidence score {max_score:.2f}"
                if is_new_record:
                    log_msg += " (NEW RECORD!)"
                self.get_logger().info(log_msg)


###############################################

        matched_boxes = [candidate_boxes[i] for i in best_labels_per_box.keys()]
        matched_labels = [f"{lbl} ({score:.2f})" for lbl, score in best_labels_per_box.values()]

        annotated_img = self.draw_boxes(cv_img, matched_boxes, matched_labels)
        try:
            self.annotated_pub.publish(self.bridge.cv2_to_imgmsg(annotated_img, encoding="bgr8"))
        except CvBridgeError as e:
            self.get_logger().error(f"Publish error: {e}")

        self.latest_rgb, self.latest_depth = None, None
        
    def match_label_box(self, label, image_tensor, boxes):
        max_sim, best_box = -1, None
        tokenized_text = clip.tokenize([label]).to(self.device)
        text_feat = F.normalize(self.clip_model.encode_text(tokenized_text), dim=-1)

        for box in boxes:
            x_min, y_min, x_max, y_max = map(int, box)
            crop = transforms.functional.to_pil_image(image_tensor[:, y_min:y_max, x_min:x_max])
            img_feat = F.normalize(self.clip_model.encode_image(self.clip_preprocess(crop).unsqueeze(0).to(self.device)), dim=-1)
            sim = F.cosine_similarity(img_feat, text_feat).item()
            if sim > max_sim:
                max_sim, best_box = sim, box
        return best_box, max_sim

    def draw_boxes(self, img, boxes, labels):
        for i, box in enumerate(boxes):
            x_min, y_min, x_max, y_max = map(int, box)
            cv2.rectangle(img, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
            if labels:
                cv2.putText(img, labels[i], (x_min, y_min-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
        return img

def main(args=None):
    rclpy.init(args=args)
    node = ClipDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Save records before exiting
        node.save_score_records()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
