#! /usr/bin/env python2.7
# -*- coding: utf-8 -*-


import json
import rospy
import tf
from geometry_msgs.msg import PolygonStamped, Point32,  PoseStamped
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rospy
import dynamic_reconfigure.client
import time


class ConfigChanger(object):

    def __init__(self):
        rospy.init_node('config_changer')

        # Var declear
        self.odom = Odometry()
        self.is_triggered = False
        self.trigger_region = PolygonStamped()
        self.position = Point32()

        # get params from parameter server
        self.odom_frame_id = rospy.get_param(
            "~odom_frame_id", "odom")
        self.global_frame_id = rospy.get_param(
            "~global_frame_id", "map")
        self.robot_frame_id = rospy.get_param(
            "~robot_frame_id", "base_link")
        self.new_config_path = rospy.get_param(
            "~new_config_path", "/home/ucar/17th_ros_ws/src/config_changer/config/new_param.json")
        try:
            with open(self.new_config_path, 'r') as f:
                text = json.loads(f.read())
            self.new_param = text['new_param']
        except KeyError as e:
            print("KeyError when parseing config file: {}".format(e))
        self.rate = rospy.get_param(
            "~rate", 10)
        pt1 = Point32()
        pt2 = Point32()
        pt3 = Point32()
        pt4 = Point32()

        pt1.x = rospy.get_param(
            "~trigger_region", [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]])[0][0]
        pt1.y = rospy.get_param(
            "~trigger_region", [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]])[0][1]
        pt2.x = rospy.get_param(
            "~trigger_region", [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]])[1][0]
        pt2.y = rospy.get_param(
            "~trigger_region", [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]])[1][1]
        pt3.x = rospy.get_param(
            "~trigger_region", [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]])[2][0]
        pt3.y = rospy.get_param(
            "~trigger_region", [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]])[2][1]
        pt4.x = rospy.get_param(
            "~trigger_region", [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]])[3][0]
        pt4.y = rospy.get_param(
            "~trigger_region", [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]])[3][1]

        self.trigger_region.header.frame_id = self.global_frame_id
        self.trigger_region.polygon.points.append(pt1)
        self.trigger_region.polygon.points.append(pt2)
        self.trigger_region.polygon.points.append(pt3)
        self.trigger_region.polygon.points.append(pt4)

        # ROS Infastructure
        self.tf_listener = tf.TransformListener()
        self.pub_trigger_region = rospy.Publisher(
            "trigger_region", PolygonStamped, queue_size=1)

    def _get_robot_pose(self):
        # lookup tf and get robot pose in map frame
        try:
            (trans, rot) = self.tf_listener.lookupTransform(
                self.global_frame_id, self.robot_frame_id, rospy.Time(0))
            self.position.x = trans[0]
            self.position.y = trans[1]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("tf lookup failed, may not switch config")

    def update_to_new_param(self):
        client = dynamic_reconfigure.client.Client(
            '/move_base/TebLocalPlannerROS')
        config = client.update_configuration(
            self.new_param['/move_base/TebLocalPlannerROS'])

        client = dynamic_reconfigure.client.Client(
            '/move_base/local_costmap')
        config = client.update_configuration(
            self.new_param['/move_base/local_costmap'])

    def is_in_triger_region(self):
        # check is robot in trigger region
        x_min = self.trigger_region.polygon.points[0].x
        x_max = self.trigger_region.polygon.points[0].x
        y_min = self.trigger_region.polygon.points[0].y
        y_max = self.trigger_region.polygon.points[0].y
        # find the x_min, y_min, x_max, y_max of the trigger region
        for point in self.trigger_region.polygon.points:
            if point.x < x_min:
                x_min = point.x
            if point.x > x_max:
                x_max = point.x
            if point.y < y_min:
                y_min = point.y
            if point.y > y_max:
                y_max = point.y

        # check is robot in trigger region
        if self.position.x >= x_min and \
                self.position.x <= x_max and \
                self.position.y >= y_min and \
                self.position.y <= y_max:
            return True
        else:
            return False


def main():
    node = ConfigChanger()
    rate = rospy.Rate(node.rate)
    while not rospy.is_shutdown():
        node.pub_trigger_region.publish(node.trigger_region)
        node._get_robot_pose()
        if not node.is_triggered:
            if node.is_in_triger_region():
                node.is_triggered = True
                print("robot is in trigger region")
                print("Start to change config")
                node.update_to_new_param()
                print("Config changed")
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit(0)
