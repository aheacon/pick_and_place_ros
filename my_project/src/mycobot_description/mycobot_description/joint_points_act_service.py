"""
Author: David Valencia
Date: 26 / 08 /2021

Describer:  An action Client to move the robot joint to a specific position

			This script send the position "angles" of each joint under 
			
			the ACTION - SERVICE /joint_trajectory_controller/joint_trajectory
			
			I need to run first the my_doosan_controller in order to load and start the controllers
			Update: I can also run my environment launch file 

			Executable name in the setup file: trajectory_points_act_server		
"""

import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class TrajectoryActionClient(Node):

    def __init__(self):

        super().__init__("points_publisher_node_action_client")
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/joint_trajectory",
        )

    def send_goal(self):
        points = []

        point1_msg = JointTrajectoryPoint()
        point1_msg.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point1_msg.time_from_start = Duration(seconds=1.0).to_msg()

        point2_msg = JointTrajectoryPoint()
        point2_msg.positions = [0.1, 0.0, 1.52, 1.5, 1.0, 0.4]
        point2_msg.time_from_start = Duration(seconds=2, nanoseconds=0).to_msg()

        point3_msg = JointTrajectoryPoint()
        point3_msg.positions = [0.3, 0.0, -0.17, 0.0, 0.0, 0.0]
        point3_msg.time_from_start = Duration(seconds=2, nanoseconds=0).to_msg()

        point4_msg = JointTrajectoryPoint()
        point4_msg.positions = [0.3, 0.0, -0.52, 0.0, 0.0, 0.0]
        point4_msg.time_from_start = Duration(seconds=2, nanoseconds=0).to_msg()

        point5_msg = JointTrajectoryPoint()
        point5_msg.positions = [0.0, 0.0, 0.17, 0.0, 0.0, 0.0]
        point5_msg.time_from_start = Duration(seconds=2, nanoseconds=0).to_msg()

        points.append(point1_msg)
        points.append(point2_msg)
        points.append(point3_msg)
        points.append(point4_msg)
        points.append(point5_msg)
        joint_names = [
            "joint2_to_joint1",
            "joint3_to_joint2",
            "joint4_to_joint3",
            "joint5_to_joint4",
            "joint6_to_joint5",
            "joint6output_to_joint6",
        ]
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.goal_time_tolerance = Duration(seconds=1, nanoseconds=0).to_msg()
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points = points
        print(f"Sending goals {goal_msg} and wait for server")
        self.action_client.wait_for_server()
        print("Server started send goals")
        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        print("Send goal")
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected ")
            return

        self.get_logger().info("Goal accepted")

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):

        result = future.result().result
        self.get_logger().info("Result: " + str(result))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback


def main(args=None):

    rclpy.init()
    print("STARTED")
    action_client = TrajectoryActionClient()
    future = action_client.send_goal()
    rclpy.spin(action_client)


if __name__ == "__main__":
    main()
