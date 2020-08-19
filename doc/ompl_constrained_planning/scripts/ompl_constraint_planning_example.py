#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg
import visualization_msgs.msg
import std_msgs.msg

from math import pi
from tf.transformations import quaternion_from_euler

COLOR_TRANSLUCENT = std_msgs.msg.ColorRGBA(0.0, 0.0, 0.0, 0.5)


class ConstrainedPlanningTutorial(object):
    def __init__(self, group_name="panda_arm"):
        # the usual setup
        # see "move_group_python_interface_tutorial" for more details
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("ompl_constrained_planning_example", anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        # create a publisher to visualize the position constraints in Rviz
        self.marker_publisher = rospy.Publisher(
            "/visualization_marker", visualization_msgs.msg.Marker, queue_size=20,
        )
        rospy.sleep(0.5)  # publisher needs some time to context Rviz

        # save some commenly used variables
        self.ref_link = self.move_group.get_pose_reference_frame()
        self.ee_link = self.move_group.get_end_effector_link()

    def create_start_state(self):
        """ Create a RobotState message from a named joint target for this robot. """

        # get a dictionary with joint names and values for the "ready" position
        ready = self.move_group.get_named_target_values("ready")

        # create the robot "ready" state
        joint_state = sensor_msgs.msg.JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.header.frame_id = self.move_group.get_pose_reference_frame()
        joint_state.name = [key for key in ready.keys()]
        joint_state.position = [val for val in ready.values()]

        state = moveit_msgs.msg.RobotState()
        state.joint_state = joint_state
        return state

    def create_position_constraints(self):
        pcm = moveit_msgs.msg.PositionConstraint()
        pcm.header.frame_id = self.ref_link
        pcm.link_name = self.ee_link

        cbox = shape_msgs.msg.SolidPrimitive()
        cbox.type = shape_msgs.msg.SolidPrimitive.BOX
        cbox.dimensions = [0.1, 0.5, 0.5]
        pcm.constraint_region.primitives.append(cbox)

        cbox_pose = geometry_msgs.msg.Pose()
        cbox_pose.position.x = 0.3
        cbox_pose.position.y = 0.0
        cbox_pose.position.z = 0.45
        cbox_pose.orientation.w = 1.0
        pcm.constraint_region.primitive_poses.append(cbox_pose)

        # display the constraints in rviz
        self.display_box(cbox_pose, cbox.dimensions, "world")
        return pcm

    def create_orientation_constraints(self):
        # use the orientation of the current robot pose as a nominal orientation
        # to center the constraints around.
        current_pose = self.move_group.get_current_pose()

        ocm = moveit_msgs.msg.OrientationConstraint()
        ocm.header.frame_id = self.ref_link
        ocm.link_name = self.ee_link
        ocm.orientation = current_pose.pose.orientation
        # ocm.orientation = geometry_msgs.msg.Quaternion(*quaternion_from_euler(0, pi, 0))
        ocm.absolute_x_axis_tolerance = 0.1
        ocm.absolute_y_axis_tolerance = 0.1
        ocm.absolute_z_axis_tolerance = 3.14

        return ocm

    def create_pose_goal(self):
        # start from the robot's current pose and go from there.
        pose = self.move_group.get_current_pose()
        pose.pose.position.y += 0.2
        pose.pose.position.z -= 0.3
        return pose

    def solve(self, start_state, pose_goal, path_constraints):
        self.move_group.set_start_state(start_state)
        self.move_group.set_pose_target(pose_goal)
        self.move_group.set_path_constraints(path_constraints)
        plan = self.move_group.plan()
        self.move_group.clear_path_constraints()

    def display_box(self, pose, dimensions, reference_frame):
        assert len(dimensions) == 3

        # setup cube / box marker type
        marker = visualization_msgs.msg.Marker()
        marker.header.stamp = rospy.Time.now()
        marker.ns = "/"
        marker.id = 0
        marker.type = visualization_msgs.msg.Marker.CUBE
        marker.action = visualization_msgs.msg.Marker.ADD
        marker.color = COLOR_TRANSLUCENT

        # fill in user input
        marker.header.frame_id = reference_frame
        marker.pose = pose
        marker.scale.x = dimensions[0]
        marker.scale.y = dimensions[1]
        marker.scale.z = dimensions[2]

        # publish it!
        self.marker_publisher.publish(marker)


def run_tutorial():
    tutorial = ConstrainedPlanningTutorial()

    start_state = tutorial.create_start_state()
    pcm = tutorial.create_position_constraints()
    ocm = tutorial.create_orientation_constraints()
    pose_goal = tutorial.create_pose_goal()
    print(pose_goal)

    path_constraints = moveit_msgs.msg.Constraints()
    path_constraints.orientation_constraints.append(ocm)
    # path_constraints.position_constraints.append(pcm)

    tutorial.solve(start_state, pose_goal, path_constraints)


def main():
    try:
        run_tutorial()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()

