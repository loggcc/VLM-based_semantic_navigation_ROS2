#!/usr/bin/env python3
import json
import clip
import torch
from copy import deepcopy
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from ament_index_python.packages import get_package_share_directory
class SemanticNavNode(Node):
    def __init__(self, json_path: str):
        super().__init__('semantic_nav_node')

        # Load object library
        with open(json_path) as f:
            self.object_lib = json.load(f)

        # Load CLIP
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model, _ = clip.load("ViT-B/32", device=self.device)

        # Build text embeddings for all object names
        self.object_names = list(self.object_lib.keys())
        text_tokens = clip.tokenize(self.object_names).to(self.device)
        with torch.no_grad():
            text_features = self.model.encode_text(text_tokens)
            self.text_features = text_features / text_features.norm(dim=-1, keepdim=True)

        # Initialize navigator
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Navigator ready")

    def query_object(self, prompt: str):
        tokens = clip.tokenize([prompt]).to(self.device)
        with torch.no_grad():
            query_feat = self.model.encode_text(tokens)
            query_feat /= query_feat.norm(dim=-1, keepdim=True)

        sims = (query_feat @ self.text_features.T).squeeze(0)
        best_idx = sims.argmax().item()
        best_name = self.object_names[best_idx]

        pos = self.object_lib[best_name].get("position")
        return best_name, pos

    def navigate_to_object(self, prompt: str):
        name, pos = self.query_object(prompt)
        if pos is None:
            self.get_logger().warn(f"No known position for object: {name}")
            return

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = pos["x"]
        goal_pose.pose.position.y = pos["y"]
        goal_pose.pose.position.z = pos.get("z", 0.0)
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0

        self.navigator.followWaypoints([deepcopy(goal_pose)])
        self.get_logger().info(f"Navigating to {name} at {pos}")

        # Wait until navigation completes
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(
                    f"Executing waypoint {feedback.current_waypoint + 1}/1"
                )

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Navigation succeeded")
        elif result == TaskResult.CANCELED:
            self.get_logger().info("Navigation canceled")
        elif result == TaskResult.FAILED:
            self.get_logger().info("Navigation failed")
            
            



def main():
    rclpy.init()
    package_share_dir = get_package_share_directory('vlm_nav_pkg')
    json_file_path = os.path.join(package_share_dir, 'config', 'example_object_detection_vlm.json')
    node = SemanticNavNode(json_file_path)

    # Example: user input from terminal
    user_input = input("Where do you want to go: ")
    node.navigate_to_object(user_input)

    rclpy.shutdown()


if __name__ == "__main__":
    main()

