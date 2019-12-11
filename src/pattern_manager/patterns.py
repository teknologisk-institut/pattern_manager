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

# Author: Mads Vainoe Baatrup

import rospy
import numpy as np
import tf.transformations as tfs

from pattern_manager.xform import XForm
from math import cos, sin, pi
from enum import Enum


class Pattern(Enum):
    """
    An enum representing the types of patterns

    """

    linear = 1
    rectangular = 2
    circular = 3
    scatter = 4


def _handle_input_1d(number_of_points=0, step_size=0, line_length=0):
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


def _matrix_to_tf(matrix):
    """
    Convert a 3x4 numpy transformation matrix to a geometry_msgs.Transform.

    :param matrix: 3x4 Transformation matrix to convert
    :type matrix: numpy.ndarray
    :return: Converted Transform
    :rtype: geometry_msgs.Transform
    """

    t = XForm()
    t.translation.x = matrix[0, 3]
    t.translation.y = matrix[1, 3]
    t.translation.z = matrix[2, 3]

    q = tfs.quaternion_from_matrix(matrix)

    t.rotation.x = q[0]
    t.rotation.y = q[1]
    t.rotation.z = q[2]
    t.rotation.w = q[3]

    return t


def create_pattern(type_, parent, args):
    """
    This interface function creates a pattern of XForms in the shape of the specified type

    :param type_: Pattern type specifying the pattern to create
    :type type_: Pattern
    :param parent: The parent XForm which will contain the pattern XForms as children
    :type parent: XForm
    :param args: A list of arguments required for the creation of a pattern respectively
    :type args: list
    """

    if type_ == Pattern.linear:
        try:
            _create_linear_pattern(parent, *args)
        except ValueError, e:
            rospy.logwarn('Bad parameters: %s' % e)
    elif type_ == Pattern.rectangular:
        try:
            _create_rectangular_pattern(parent, *args)
        except ValueError, e:
            rospy.logwarn('Bad parameters: %s' % e)
    elif type_ == Pattern.circular:
        try:
            _create_circular_pattern(parent, *args)
        except ValueError, e:
            rospy.logwarn('Bad parameters: %s' % e)
    elif type_ == Pattern.scatter:
        try:
            _create_scatter_pattern(parent, *args)
        except ValueError, e:
            rospy.logwarn('Bad parameters: %s' % e)
    else:
        rospy.logerr('Pattern type %s does not exist' % type_)


def _create_linear_pattern(parent, num_points=0, step_size=0.0, line_len=0.0, axis='x'):
    """
    This function generates a pattern of XForms in a linear shape

    :param parent: The parent XForm which will contain the pattern XForms as children
    :type parent: XForm
    :param num_points: The number of points desired in the pattern
    :type num_points: int
    :param step_size: The distance between each point
    :type step_size: float
    :param line_len: The total length of the pattern
    :type line_len: float
    :param axis: The axis along which to generate the pattern
    :type axis: str
    """

    try:
        (po, st, le) = _handle_input_1d(num_points, step_size, line_len)
    except TypeError, e:
        rospy.logerr(e)

        return None

    x_set = set()

    for i in range(po):
        x_set.add(i * st)

    c = 0
    for x in x_set:
        tf = XForm(parent, name='{}_{}'.format(parent.name, c))
        tf.translation.x = x

        c += 1


def _create_rectangular_pattern(parent, num_points=(0, 0), step_sizes=(0, 0), line_lens=(0.0, 0.0)):
    """
    This function generates a pattern of XForms in a rectangular shape

    :param parent: The parent XForm which will contain the pattern XForms as children
    :type parent: XForm
    :param num_points: A tuple of the number of points along each axis of the pattern
    :type num_points: tuple
    :param step_sizes: A tuple of the distances between between points along each axis
    :type step_sizes: tuple
    :param line_lens: A tuple of the total lengths along each axis of the pattern
    :type line_lens: tuple
    """

    try:
        (po_x, st_x, le_x) = _handle_input_1d(num_points[0], step_sizes[0], line_lens[0])
        (po_y, st_y, le_y) = _handle_input_1d(num_points[1], step_sizes[1], line_lens[1])
    except TypeError, e:
        rospy.logerr(e)

        return None

    xy_set = set()

    for i in range(po_x):
        for j in range(po_y):
            xy_set.add((i * st_x, j * st_y))

    c = 0
    for xy in xy_set:
        tf = XForm(parent, name='{}_{}'.format(parent.name, c))
        tf.translation.x = xy[0]
        tf.translation.y = xy[1]

        c += 1


def _create_scatter_pattern(parent, points):
    """
    This function generates a pattern of XForms positioned according to the specified list of points

    :param parent: The parent XForm which will contain the pattern XForms as children
    :type parent: XForm
    :param points: A list of points specifying the positions of each transform of the pattern
    :type points: list
    """

    if not len(points) > 0:
        rospy.logerr("Scatter points cannot be zero")

        return None

    xyz_set = set()

    for p in points:
        xyz_set.add((p[0], p[1], p[2]))

    i = 0
    for xyz in xyz_set:
        tf = XForm(parent, name='{}_{}'.format(parent.name, i))

        tf.translation.x = xyz[0]
        tf.translation.y = xyz[1]
        tf.translation.z = xyz[2]

        i += 1


def _create_circular_pattern(parent, num_points=0, r=0.0, tan_rot=False, cw=False, angular_section=2*pi):
    """
    This function generates a pattern of XForms in a circular shape

    :param parent: The parent XForm which will contain the pattern XForms as children
    :type parent: XForm
    :param num_points: The number of points to generate the circular pattern from
    :type num_points: int
    :param r: The radius of the circular pattern
    :type r: float
    :param tan_rot: Specifies whether to rotate each point tangent to the circle
    :type tan_rot: bool
    :param cw: Specifies whether to spawn the circular pattern counter-clockwise
    :type cw: bool
    :param angular_section: The size of a angular section
    :type angular_section: float
    """

    if abs(r) > 0:
        if num_points > 0:
            if angular_section == 0 or abs(angular_section - 2 * pi) < 0.01:
                angular_section = 2 * pi
        else:
            rospy.logerr("Number of points is 0, cannot define a circular pattern")
    else:
        rospy.logerr("A radius of 0 is specified, cannot define a circular pattern")

    angular_resolution = angular_section / num_points

    if cw:
        angular_resolution *= -1

    if not angular_section == 2 * pi:
        num_points += 1

    xyz_set = set()

    for i in range(num_points):
        xyz_set.add((r * cos(i * angular_resolution), r * sin(i * angular_resolution), 0.0))

    c = 0
    for xyz in xyz_set:
        t = XForm(parent, name='{}_{}'.format(parent.name, c))
        t.translation.x = xyz[0]
        t.translation.y = xyz[1]
        t.translation.z = xyz[2]

        if tan_rot:
            yaw = pi / 2 + c * angular_resolution
            m = tfs.euler_matrix(0, 0, yaw)
            q = _matrix_to_tf(m).rotation
            t.rotation = q
        else:
            t.rotation.w = 1.0

        c += 1
