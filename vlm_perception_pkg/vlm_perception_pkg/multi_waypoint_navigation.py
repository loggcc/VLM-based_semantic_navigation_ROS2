#!/usr/bin/env python3
from copy import deepcopy
from geometry_msgs.msg import PoseStamped
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator

def main():
    rclpy.init()
    navigator = BasicNavigator()
    inspection_route = [
        [5.0, 0.0],
        [-5.0, -5.0],
        [-5.0, 5.0]
    ]

    print("get nav points")
    navigator.waitUntilNav2Active()

    inspection_points = []
    inspection_pose = PoseStamped()
    inspection_pose.header.frame_id = 'map'
    inspection_pose.header.stamp = navigator.get_clock().now().to_msg()
    inspection_pose.pose.orientation.z = 1.0
    inspection_pose.pose.orientation.w = 0.0

    for pt in inspection_route:
        inspection_pose.pose.position.x = pt[0]
        inspection_pose.pose.position.y = pt[1]
        inspection_points.append(deepcopy(inspection_pose))

    navigator.followWaypoints(inspection_points)

    i = 0
    while not navigator.isNavComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('executing waypoint: ' +
                  str(feedback.current_waypoint + 1) + '/' +
                  str(len(inspection_points)))

    result = navigator.get_result()
    if result == navigator.SUCCEEDED:
        print("Navigation succeeded")
    elif result == navigator.CANCELED:
        print("Navigation canceled")
    elif result == navigator.FAILED:
        print("Navigation failed")

if __name__ == "__main__":
    main()
