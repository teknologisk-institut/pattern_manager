#!/usr/bin/env python

# Copyright 2019 Danish Technological Institute (DTI)

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Author: Mads Vainoe Baatrup, Mikkel Rath Hansen

from __future__ import division
from pattern_manager.xform import XForm
from visualization_msgs.msg import Marker, MarkerArray

import tf.transformations as tfs
import rospy


def handle_input_1d(number_of_points=0, step_size=0, line_length=0):
    """
    Generates 1D spatial information from 3 inputs.

    If any pair of inputs is specified, caluclates the third corresponding parameter. If 3 or a single input is specified, throws an error.

    :param number_of_points: Number of points along the 1D axis, defaults to 0
    :type number_of_points: float, optional
    :param step_size: Step size between points on the axis, defaults to 0
    :type step_size: float, optional
    :param line_length: Length of the axis, between first and last point, defaults to 0
    :type line_length: float, optional
    :return: 3-tuple of number of points, step size between points, and distance from first to last point.
    :rtype: tuple
    """
    out_p = 0  # number of points
    out_s = 0.0  # step size
    out_l = 0.0  # line length

    # a pair must be specified
    if number_of_points == step_size == line_length == 0:
        rospy.logerr("1D - No parameters specified")

        return False
    elif 0 not in [number_of_points, step_size, line_length]:
        rospy.logerr("1D - Ambiguous parameters, all three specified (number_of_points, step_size, "
                     "line_length)")

        return False
    elif number_of_points == 0 and not step_size == line_length == 0:
        out_s = step_size
        out_l = line_length
        # calculate points
        p = line_length / step_size
        out_p = int(p + 1)

        return out_p, out_s, out_l
    elif step_size == 0 and not number_of_points == line_length == 0:
        out_l = line_length
        out_p = number_of_points
        # calculate step
        out_s = line_length / (number_of_points - 1)

        return out_p, out_s, out_l
    elif line_length == 0 and not number_of_points == step_size == 0:
        out_s = step_size
        out_p = number_of_points
        # calculate length
        out_l = step_size * (number_of_points - 1)

        return out_p, out_s, out_l


def matrix_to_tf(matrix):
    """
    Convert a 3x4 numpy transformation matrix to a geometry_msgs.Transform.

    :param matrix: 3x4 Transformation matrix to convert
    :type matrix: numpy.ndarray
    :return: Converted Transform
    :rtype: geometry_msgs.Transform
    """

    t = XForm(None, '')
    t.translation.x = matrix[0, 3]
    t.translation.y = matrix[1, 3]
    t.translation.z = matrix[2, 3]

    q = tfs.quaternion_from_matrix(matrix)

    t.rotation.x = q[0]
    t.rotation.y = q[1]
    t.rotation.z = q[2]
    t.rotation.w = q[3]

    return t


def broadcast_transforms(br, xfs):
    """
    This function is responsible for broadcasting the XForms translation and rotation via tf

    :param br: The transform broadcaster
    :type br: tf.TransformBroadcaster
    :param xfs: A list of XForms to broadcast
    :type xfs: list
    """

    for xf in xfs:
        br.sendTransform(
            [
                xf.translation.x,
                xf.translation.y,
                xf.translation.z
            ],
            [
                xf.rotation.x,
                xf.rotation.y,
                xf.rotation.z,
                xf.rotation.w
            ],
            rospy.Time.now(),
            xf.name,
            xf.ref_frame)


def publish_markers(pub, xfs, root):
    """
    This function is responsible for publishing markers for each XForm

    :param pub: The ROS publisher object which publishes each marker in a marker array
    :type pub: rospy.Publisher
    :param xfs: A list of XForms to create markers for
    :type xfs: list
    """

    arr = MarkerArray()

    id_ = 0
    for xf in xfs:
        marker = Marker()
        marker.header.frame_id = xf.ref_frame
        marker.header.stamp = rospy.Time.now()
        marker.id = id_
        marker.type = Marker.SPHERE
        marker.action = marker.ADD
        marker.pose.position.x = xf.translation.x
        marker.pose.position.y = xf.translation.y
        marker.pose.position.z = xf.translation.z
        marker.pose.orientation.x = xf.rotation.x
        marker.pose.orientation.y = xf.rotation.y
        marker.pose.orientation.z = xf.rotation.z
        marker.pose.orientation.w = xf.rotation.w
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        r = g = b = 0.0
        if id(root.get_current_node()) == id(xf):
            g = 1.0
        elif xf.active:
            r = 1.0
            g = 1.0

        marker.color.a = 1.0
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b

        arr.markers.append(marker)

        id_ += 1

    pub.publish(arr)
