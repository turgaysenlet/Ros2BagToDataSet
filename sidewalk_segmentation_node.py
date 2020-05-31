#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from gps_msgs.msg import GPSFix
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Quaternion, TransformStamped
from cv_bridge import CvBridge, CvBridgeError
import pyrealsense2 as rs
# import tf2_ros
# from tf2_ros.transform_broadcaster import TransformBroadcaster
import numpy as np
import cv2
import logging
import sys
import json
import yaml
import base64
import re


class SidewalkSegmentation(Node):
    python_to_ros_type_map = {
        'bool': ['bool'],
        'int': ['int8', 'byte', 'uint8', 'char',
                'int16', 'uint16', 'int32', 'uint32',
                'int64', 'uint64', 'float32', 'float64'],
        'float': ['float32', 'float64'],
        'str': ['string'],
        'unicode': ['string'],
        'long': ['uint64']
    }
    python_string_types = [str]
    python_list_types = [list, tuple]

    ros_time_types = ['time', 'duration']
    ros_primitive_types = ['bool', 'byte', 'char', 'int8', 'uint8', 'int16',
                           'uint16', 'int32', 'uint32', 'int64', 'uint64',
                           'float32', 'float64', 'string']
    ros_header_types = ['Header', 'std_msgs/Header', 'roslib/Header']
    ros_binary_types_regexp = re.compile(r'(uint8|char)\[[^\]]*\]')

    list_brackets = re.compile(r'\[[^\]]*\]')
    path = '/home/turgay/Data/Image/Set1'

    def __init__(self):
        super().__init__('sidewalk_segmentation_node')
        self.get_logger().info('Init node...')
        self.bridge = CvBridge()

        self.color_subscription = self.create_subscription(
            Image,
            '/d435/camera/color/image_raw',
            self.color_listener_callback,
            10)
        self.color_subscription  # prevent unused variable warning

        self.depth_subscription = self.create_subscription(
            Image,
            '/d435/camera/aligned_depth_to_color/image_raw',
            self.depth_listener_callback,
            10)
        self.depth_subscription  # prevent unused variable warning

        self.fish1_subscription = self.create_subscription(
            Image,
            '/t265/camera/fisheye1/image_raw',
            self.fish1_listener_callback,
            10)
        self.fish1_subscription  # prevent unused variable warning

        self.fish2_subscription = self.create_subscription(
            Image,
            '/t265/camera/fisheye2/image_raw',
            self.fish2_listener_callback,
            10)
        self.fish2_subscription  # prevent unused variable warning

        self.gps_subscription = self.create_subscription(
            GPSFix,
            '/gps_fix',
            self.gps_listener_callback,
            10)
        self.gps_subscription  # prevent unused variable warning

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_listener_callback,
            10)
        self.odom_subscription  # prevent unused variable warning

        self.get_logger().info('Init node.')

    def _convert_from_ros_type(self, field_type, field_value):
        if self.is_ros_binary_type(field_type, field_value):
            field_value = self._convert_from_ros_binary(field_type, field_value)
        elif field_type in self.ros_time_types:
            field_value = self._convert_from_ros_time(field_type, field_value)
        elif field_type in self.ros_primitive_types:
            field_value = field_value
        elif self._is_field_type_a_primitive_array(field_type):
            field_value = list(field_value)
        elif self._is_field_type_an_array(field_type):
            field_value = self._convert_from_ros_array(field_type, field_value)
        else:
            field_value = self.convert_ros_message_to_dictionary(field_value)

        return field_value

    def is_ros_binary_type(self, field_type, field_value):
        """ Checks if the field is a binary array one, fixed size or not
        is_ros_binary_type("uint8", 42)
         False
        is_ros_binary_type("uint8[]", [42, 18])
         True
        is_ros_binary_type("uint8[3]", [42, 18, 21]
         True
        is_ros_binary_type("char", 42)
         False
        is_ros_binary_type("char[]", [42, 18])
         True
        is_ros_binary_type("char[3]", [42, 18, 21]
         True
        """
        return re.search(self.ros_binary_types_regexp, field_type) is not None

    def _convert_from_ros_binary(self, field_type, field_value):
        field_value = base64.standard_b64encode(field_value).decode('utf-8')
        return field_value

    def _convert_from_ros_time(self, field_type, field_value):
        field_value = {
            'secs': field_value.secs,
            'nsecs': field_value.nsecs
        }
        return field_value

    def _convert_from_ros_array(self, field_type, field_value):
        list_type = self.list_brackets.sub('', field_type)
        return [self._convert_from_ros_type(list_type, value) for value in field_value]

    def _get_message_fields(self, message):
        # return zip(message.__slots__, message._slot_types)
        return zip(message.__slots__)

    def convert_ros_message_to_dictionary(self, msg):
        """
        Takes in a ROS message and returns a Python dictionary.
        Example:
            ros_message = std_msgs.msg.String(data="Hello, Robot")
            dict_message = convert_ros_message_to_dictionary(ros_message)
        """
        dictionary = {}
        message_fields = self._get_message_fields(msg)
        for field_name, field_type in message_fields:
            field_value = getattr(msg, field_name)
            dictionary[field_name] = self._convert_from_ros_type(field_type, field_value)

        return dictionary

    def odom2json(self, msg):
        s = '{"position_x": ' + str(msg.pose.pose.position.x) + ','
        s = s + '"position_y": ' + str(msg.pose.pose.position.y) + ','
        s = s + '"position_z": ' + str(msg.pose.pose.position.z) + ','
        s = s + '"orientation_x": ' + str(msg.pose.pose.orientation.x) + ','
        s = s + '"orientation_y": ' + str(msg.pose.pose.orientation.y) + ','
        s = s + '"orientation_z": ' + str(msg.pose.pose.orientation.z) + ','
        s = s + '"orientation_w": ' + str(msg.pose.pose.orientation.w) + ','
        s = s + '"twist_linear_x": ' + str(msg.twist.twist.linear.y) + ','
        s = s + '"twist_linear_y": ' + str(msg.twist.twist.linear.y) + ','
        s = s + '"twist_linear_z": ' + str(msg.twist.twist.linear.z) + ','
        s = s + '"twist_angular_x": ' + str(msg.twist.twist.angular.x) + ','
        s = s + '"twist_angular_y": ' + str(msg.twist.twist.angular.y) + ','
        s = s + '"twist_angular_z": ' + str(msg.twist.twist.angular.z)
        s = s + '}'
        return json.dumps(json.loads(s), indent=4, sort_keys=True)

    def gps2json(self, msg):
        s = '{"latitude": ' + str(msg.latitude) + ','
        s = s + '"longitude": ' + str(msg.longitude) + ','
        s = s + '"altitude": ' + str(msg.altitude) + ','
        s = s + '"speed": ' + str(msg.speed) + ','
        s = s + '"time": ' + str(msg.time) + ','
        s = s + '"track": ' + str(msg.track) + ','
        s = s + '"pitch": ' + str(msg.pitch) + ','
        s = s + '"roll": ' + str(msg.roll) + ','
        s = s + '"dip": ' + str(msg.dip) + ','
        s = s + '"status": ' + str(msg.status.status) + ','
        s = s + '"satellites_used": ' + str(msg.status.satellites_used)
        s = s + '}'
        return json.dumps(json.loads(s), indent=4, sort_keys=True)

    def msg2json(self, msg):
        """
        Takes in a ROS message and returns a JSON-formatted string.
        Example:
            ros_message = std_msgs.msg.String(data="Hello, Robot")
            json_message = convert_ros_message_to_json(ros_message)
        """
        dictionary = self.convert_ros_message_to_dictionary(msg)
        json_message = json.dumps(dictionary)
        return json_message

    def msg2yaml(self, msg):
        # Convert a ROS message to JSON format
        self.get_logger().info(str(msg))
        y = yaml.load(str(msg))
        self.get_logger().info(y)
        return json.dumps(y, indent=4, sort_keys=True)

    def gps_listener_callback(self, msg):
        self.save_gps_json(msg, 'gps')
        return

    def odom_listener_callback(self, msg):
        self.save_odom_json(msg, 'odom')

    def save_odom_json(self, msg, prefix):
        filename = self.get_filename(msg, prefix, 'json')
        self.get_logger().info(filename)
        s = self.odom2json(msg)
        file = open(filename, "w+")
        file.write(s)
        file.close()

    def save_gps_json(self, msg, prefix):
        filename = self.get_filename(msg, prefix, 'json')
        self.get_logger().info(filename)
        s = self.gps2json(msg)
        file = open(filename, "w+")
        file.write(s)
        file.close()

    def get_filename(self, msg, prefix, extension):
        s = "{:.10f}".format(msg.header.stamp.nanosec / 1000000000)
        s = s[1:12]
        return '{}/{}_{}{}.{}'.format(self.path, prefix, msg.header.stamp.sec, s, extension)

    def save_image(self, msg, prefix, image_type):
        filename = self.get_filename(msg, prefix, 'png')
        image = self.bridge.imgmsg_to_cv2(msg, image_type)
        self.get_logger().info(filename)
        cv2.imwrite(filename, image)

    def fish1_listener_callback(self, msg):
        self.save_image(msg, 'fisheye1', '8UC1')

    def fish2_listener_callback(self, msg):
        self.save_image(msg, 'fisheye2', '8UC1')

    def depth_listener_callback(self, msg):
        self.save_image(msg, 'depth', '16UC1')

    def color_listener_callback(self, msg):
        self.save_image(msg, 'color', 'bgr8')

        # lower = image[int(image.shape[0] / 2):, :]
        # gray_lower = cv2.cvtColor(lower, cv2.COLOR_BGR2GRAY)
        #
        # blur = cv2.GaussianBlur(gray_lower, (5, 5), 0)
        # ret, otsu = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        # cv2.imshow('image', image)
        # cv2.imshow('gray_lower', gray_lower)
        # cv2.imshow('otsu', otsu)
        #
        # cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    sidewalk_segmentation_node = SidewalkSegmentation()

    rclpy.spin(sidewalk_segmentation_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sidewalk_segmentation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
