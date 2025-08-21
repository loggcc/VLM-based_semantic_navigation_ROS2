#!/usr/bin/env python3
from copy import deepcopy
from geometry_msgs.msg import PoseStamped
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def main():
    rclpy.init()
    navigator = BasicNavigator()
    
    inspection_route = [
        [-3.59, 2.93],
        [-3.59, 2.93],
        [-3.59, 2.98]
    ]

    # Wait for Nav2 to be fully active
    print("Waiting for Nav2 to be active...")
    navigator.waitUntilNav2Active() # This waits for the lifecycle nodes, AMCL, etc.
    print("Nav2 is active!")

    # ******************************************
    # REMOVED: The initial_pose code block
    # You will set this manually in RViz
    # ******************************************

    # Create waypoints
    
        

    inspection_points = []
    for pt in inspection_route:
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = pt[0]
        goal_pose.pose.position.y = pt[1]
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        inspection_points.append(deepcopy(goal_pose))


    print(f"Following {len(inspection_points)} waypoints...")
    navigator.followWaypoints(inspection_points)

    
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(f'Executing waypoint: {feedback.current_waypoint + 1}/{len(inspection_points)}')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("Navigation succeeded!")
    elif result == TaskResult.CANCELED:
        print("Navigation canceled!")
    elif result == TaskResult.FAILED:
        print("Navigation failed!")

    rclpy.shutdown()

if __name__ == "__main__":
    main()
