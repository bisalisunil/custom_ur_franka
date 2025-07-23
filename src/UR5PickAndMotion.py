#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
from geometry_msgs.msg import Pose

class UR5PickAndMotion(Node):
    def __init__(self):
        super().__init__('ur5_pick_and_motion_node')
        self.get_logger().info("Starting UR5 Pick and Motion Node")

        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        
        # // # Define MoveGroup for UR5
        self.group = MoveGroupCommander("ur5_manipulator")

        # // # ✅ Set the correct end-effector link
        self.group.set_end_effector_link("robotiq_85_base_link")

        # Just for debug
        self.get_logger().info(f"End effector link set to: {self.group.get_end_effector_link()}")

        self.group.set_pose_reference_frame("base_link")
        self.group.set_planning_time(10.0)

        self.execute_pick_and_motion()

    def execute_pick_and_motion(self):
        pick_pose = Pose()
        pick_pose.position.x = 0.4
        pick_pose.position.y = 0.0
        pick_pose.position.z = 0.3
        pick_pose.orientation.w = 1.0

        self.group.set_pose_target(pick_pose)

        success = self.group.go(wait=True)
        if not success:
            self.get_logger().error("Failed to move to pick pose")
            return
        self.group.stop()
        self.group.clear_pose_targets()

        # Move in a circular motion
        waypoints = []

        center_x = pick_pose.position.x
        center_y = pick_pose.position.y
        center_z = pick_pose.position.z

        for angle in range(0, 360, 30):
            wpose = Pose()
            wpose.position.x = center_x + 0.05 * np.cos(np.radians(angle))
            wpose.position.y = center_y + 0.05 * np.sin(np.radians(angle))
            wpose.position.z = center_z
            wpose.orientation.w = 1.0
            waypoints.append(wpose)

        (plan, fraction) = self.group.compute_cartesian_path(
            waypoints, 0.01, 0.0
        )

        if fraction < 0.9:
            self.get_logger().warn("Incomplete circular path")
        else:
            self.group.execute(plan, wait=True)
            self.get_logger().info("Completed circular motion")

def main(args=None):
    rclpy.init(args=args)
    node = UR5PickAndMotion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    import numpy as np
    main()
