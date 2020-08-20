#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2020, KU Leuven
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Jeroen De Maeyer

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
from six.moves import input  # Python 3 compatible alternative for raw_input

COLOR_RED = std_msgs.msg.ColorRGBA(1.0, 0.0, 0.0, 1.0)
COLOR_GREEN = std_msgs.msg.ColorRGBA(0.0, 1.0, 0.0, 1.0)
COLOR_TRANSLUCENT = std_msgs.msg.ColorRGBA(0.0, 0.0, 0.0, 0.5)


# BEGIN_SUB_TUTORIAL setup
##
# Setup a RobotCommander and a MoveGroupCommander, see "move_group_python_interface_tutorial" for more details.
# Everything is wrappen a a class so we can easily reuse the different components.
##
class ConstrainedPlanningTutorial(object):
    def __init__(self, group_name="panda_arm"):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("ompl_constrained_planning_example", anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.scene = moveit_commander.PlanningSceneInterface()

        # Create a publisher to visualize the position constraints in Rviz
        self.marker_publisher = rospy.Publisher(
            "/visualization_marker", visualization_msgs.msg.Marker, queue_size=20,
        )
        rospy.sleep(0.5)  # publisher needs some time to context Rviz
        self.remove_all_markers()
        self.marker_id_counter = 0  # give each marker a unique idea

        # Save some commenly used variables in the setup class
        self.ref_link = self.move_group.get_pose_reference_frame()
        self.ee_link = self.move_group.get_end_effector_link()
        self.obstacle_name = "obstacle"

    # END_SUB_TUTORIAL

    # BEGIN_SUB_TUTORIAL start_state
    # Instead of using the current robot state as start state, we use a fixed state for the panda robot defined in its `srdf` config file.
    # The `get_named_target_values` returns a dictionary with joint names and values for the "ready" position.
    def create_start_state(self):
        """ Create a RobotState message from a named joint target for this robot. """
        ready = self.move_group.get_named_target_values("ready")

        # Now create a robot state from these joint positions
        joint_state = sensor_msgs.msg.JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.header.frame_id = self.move_group.get_pose_reference_frame()
        joint_state.name = [key for key in ready.keys()]
        joint_state.position = [val for val in ready.values()]

        state = moveit_msgs.msg.RobotState()
        state.joint_state = joint_state

        return state

    # END_SUB_TUTORIAL

    # BEGIN_SUB_TUTORIAL pose_goal
    ##
    # To keep things simple, we use the current end-effector pose to quickly create a reasonable goal.
    # We also visualize the start and goal position of the end-effector in Rviz with a simple sphere.
    # We assume that when the `demo.launch` file for the panda robot is launched, the robot is in the "ready" position.
    def create_pose_goal(self):
        self.move_group.clear_pose_targets()
        pose = self.move_group.get_current_pose()

        self.display_sphere(pose.pose, color=COLOR_RED)

        pose.pose.position.y += 0.3
        pose.pose.position.z -= 0.3

        self.display_sphere(pose.pose)

        return pose

    def create_pose_goal_2(self):
        self.move_group.clear_pose_targets()
        pose = self.move_group.get_current_pose()

        self.display_sphere(pose.pose, color=COLOR_RED)

        pose.pose.position.x += 0.2
        pose.pose.position.y += 0.3
        pose.pose.position.z -= 0.3

        self.display_sphere(pose.pose)

        return pose

    # END_SUB_TUTORIAL

    # BEGIN_SUB_TUTORIAL ori_con
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

    # END_SUB_TUTORIAL

    # BEGIN_SUB_TUTORIAL pos_con
    def create_position_constraints(self):
        pcm = moveit_msgs.msg.PositionConstraint()
        pcm.header.frame_id = self.ref_link
        pcm.link_name = self.ee_link

        cbox = shape_msgs.msg.SolidPrimitive()
        cbox.type = shape_msgs.msg.SolidPrimitive.BOX
        cbox.dimensions = [0.1, 0.3, 0.5]
        pcm.constraint_region.primitives.append(cbox)

        current_pose = self.move_group.get_current_pose()

        cbox_pose = geometry_msgs.msg.Pose()
        cbox_pose.position.x = current_pose.pose.position.x
        cbox_pose.position.y = 0.1
        cbox_pose.position.z = 0.45
        cbox_pose.orientation.w = 1.0
        pcm.constraint_region.primitive_poses.append(cbox_pose)

        # display the constraints in rviz
        self.display_box(cbox_pose, cbox.dimensions)

        return pcm

    def create_position_constraints_3(self):
        pcm = moveit_msgs.msg.PositionConstraint()
        pcm.header.frame_id = self.ref_link
        pcm.link_name = self.ee_link

        cbox = shape_msgs.msg.SolidPrimitive()
        cbox.type = shape_msgs.msg.SolidPrimitive.BOX
        cbox.dimensions = [0.005, 0.005, 2.0]
        pcm.constraint_region.primitives.append(cbox)

        current_pose = self.move_group.get_current_pose()

        cbox_pose = geometry_msgs.msg.Pose()
        cbox_pose.position.x = current_pose.pose.position.x
        cbox_pose.position.y = current_pose.pose.position.y
        cbox_pose.position.z = current_pose.pose.position.z
        quat = quaternion_from_euler(pi/4, 0, 0)
        cbox_pose.orientation.x = quat[0]
        cbox_pose.orientation.y = quat[1]
        cbox_pose.orientation.z = quat[2]
        cbox_pose.orientation.w = quat[3]
        pcm.constraint_region.primitive_poses.append(cbox_pose)

        # display the constraints in rviz
        self.display_box(cbox_pose, cbox.dimensions)

        return pcm

    def create_position_constraints_2(self):
        pcm = moveit_msgs.msg.PositionConstraint()
        pcm.header.frame_id = self.ref_link
        pcm.link_name = self.ee_link

        cbox = shape_msgs.msg.SolidPrimitive()
        cbox.type = shape_msgs.msg.SolidPrimitive.BOX
        cbox.dimensions = [1.0, 0.001, 1.0]
        pcm.constraint_region.primitives.append(cbox)

        current_pose = self.move_group.get_current_pose()

        cbox_pose = geometry_msgs.msg.Pose()
        cbox_pose.position.x = current_pose.pose.position.x
        cbox_pose.position.y = current_pose.pose.position.y
        cbox_pose.position.z = current_pose.pose.position.z

        quat = quaternion_from_euler(pi/4, 0, 0)
        cbox_pose.orientation.x = quat[0]
        cbox_pose.orientation.y = quat[1]
        cbox_pose.orientation.z = quat[2]
        cbox_pose.orientation.w = quat[3]
        pcm.constraint_region.primitive_poses.append(cbox_pose)

        # display the constraints in rviz
        self.display_box(cbox_pose, cbox.dimensions)

        return pcm

    # END_SUB_TUTORIAL

    # BEGIN_SUB_TUTORIAL solve
    # We add a simple solve function that we can use to quickly try solving a problem for different path constraints.
    def solve(self, start_state, pose_goal, path_constraints):
        self.move_group.set_start_state(start_state)
        self.move_group.set_pose_target(pose_goal)

        # Don't forget the path constraints! That's the whole point of this tutorial.
        self.move_group.set_path_constraints(path_constraints)

        # And let the planner find a solution.
        # The move_group node should autmatically visualize the solution in Rviz if a path is found.
        self.move_group.plan()

        # Clear the path constraints for our next experiment
        self.move_group.clear_path_constraints()

    # END_SUB_TUTORIAL

    def display_box(self, pose, dimensions):
        """ Utility function to visualize position constraints. """
        assert len(dimensions) == 3

        # setup cube / box marker type
        marker = visualization_msgs.msg.Marker()
        marker.header.stamp = rospy.Time.now()
        marker.ns = "/"
        marker.id = self.marker_id_counter
        marker.type = visualization_msgs.msg.Marker.CUBE
        marker.action = visualization_msgs.msg.Marker.ADD
        marker.color = COLOR_TRANSLUCENT
        marker.header.frame_id = self.ref_link

        # fill in user input
        marker.pose = pose
        marker.scale.x = dimensions[0]
        marker.scale.y = dimensions[1]
        marker.scale.z = dimensions[2]

        # publish it!
        self.marker_publisher.publish(marker)
        self.marker_id_counter += 1

    def display_sphere(self, pose, radius=0.05, color=COLOR_GREEN):
        """ Utility function to visualize the goal pose"""

        # setup sphere marker type
        marker = visualization_msgs.msg.Marker()
        marker.header.stamp = rospy.Time.now()
        marker.ns = "/"
        marker.id = self.marker_id_counter
        marker.type = visualization_msgs.msg.Marker.SPHERE
        marker.action = visualization_msgs.msg.Marker.ADD
        marker.header.frame_id = self.ref_link

        # fill in user input
        marker.color = color
        marker.pose = pose
        marker.scale.x = radius
        marker.scale.y = radius
        marker.scale.z = radius

        # publish it!
        self.marker_publisher.publish(marker)
        self.marker_id_counter += 1

    def remove_all_markers(self):
        """ Utility function to remove all Markers that we potentially published in a previous run of this script. """
        # setup cube / box marker type
        marker = visualization_msgs.msg.Marker()
        marker.header.stamp = rospy.Time.now()
        marker.ns = "/"
        # marker.id = 0
        # marker.type = visualization_msgs.msg.Marker.CUBE
        marker.action = visualization_msgs.msg.Marker.DELETEALL
        self.marker_publisher.publish(marker)


def run_tutorial():
    # BEGIN_SUB_TUTORIAL main
    ##
    # Now we can use the different pieces we just defined to solve some planning problems!
    # First create an instance and get a start_state and pose_goal
    tutorial = ConstrainedPlanningTutorial()
    start_state = tutorial.create_start_state()
    pose_goal = tutorial.create_pose_goal()

    # Now let's try the orientation constraints!
    ocm = tutorial.create_orientation_constraints()

    # We need two wrap the constraints in a generic `Constraints` message.
    path_constraints = moveit_msgs.msg.Constraints()
    path_constraints.orientation_constraints.append(ocm)

    # tutorial.add_obstacle()
    # rospy.sleep(0.1)

    # Call our simple solve function and look at Rviz to see the result.
    tutorial.solve(start_state, pose_goal, path_constraints)

    # Now wait for the user(you) to press enter before doing trying the position constraints.
    print("============ Press enter to continue with the second planning problem.")
    input()

    pcm = tutorial.create_position_constraints_3()

    path_constraints = moveit_msgs.msg.Constraints()
    path_constraints.orientation_constraints = []
    path_constraints.position_constraints.append(pcm)

    path_constraints.name = "use_equality_constraints"

    tutorial.solve(start_state, pose_goal, path_constraints)

    print("============ Press enter to continue with the second planning problem.")
    input()
    tutorial.remove_all_markers()
    pose_goal = tutorial.create_pose_goal_2()

    pcm = tutorial.create_position_constraints_2()

    path_constraints = moveit_msgs.msg.Constraints()
    path_constraints.orientation_constraints = []
    path_constraints.position_constraints.append(pcm)

    path_constraints.name = "use_equality_constraints"

    tutorial.solve(start_state, pose_goal, path_constraints)

    # END_SUB_TUTORIAL


if __name__ == "__main__":
    run_tutorial()

# BEGIN_TUTORIAL
##
# Setup
# *****
# CALL_SUB_TUTORIAL setup
###
# Create a planning problem
# ***************************
# CALL_SUB_TUTORIAL start_state
# CALL_SUB_TUTORIAL pose_goal
##
# Create orientation constraints
# ******************************
# CALL_SUB_TUTORIAL ori_con
##
# Create position constraints
# ***************************
# CALL_SUB_TUTORIAL pos_con
##
# Create a simple solve function
# ******************************
# CALL_SUB_TUTORIAL solve
##
# Finally, solve a planning problem!
# **********************************
# CALL_SUB_TUTORIAL main
##
# END_TUTORIAL
