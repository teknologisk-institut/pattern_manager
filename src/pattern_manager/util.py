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

from __future__ import division
from copy import deepcopy
from pattern_manager.xform import XForm
from math import cos, sin, pi

import geometry_msgs.msg as gm
import numpy as np
import tf.transformations as tfs
import rospy
# import logging
# logging.basicConfig(format='(Pattern Manager) %(levelname)s: %(message)s', level=logging.DEBUG)
# output = logging


# Helper functions
def handle_input_1d(number_of_points=0, step_size=0, line_length=0):
    """Generates 1D spatial information from 3 inputs.

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


def tfs_along_axis(count, step_size, parent, basis_frame=None, axis='x'):
    """Generate a series of given number of frames along one axis, with given distance between frames.
    
    :param count: Number of frames to generate
    :type count: int
    :param step_size: Distance between frames, in m
    :type step_size: float
    :param basis_frame: Frame to use as basis for generation, defaults to empty geometry_msgs.Transform()
    :type basis_frame: geometry_msgs.Transform optional
    :param axis: Which axis to generate the frames along, defaults to 'x'
    :type axis: str, optional
    :return: List of the generated frames. 
    :rtype: numpy.array with dtype geometry_msgs.Transform
    """
    frames = np.array(np.empty(count), dtype=XForm)
    for p in range(count):
        transf = XForm(parent) if not basis_frame else deepcopy(basis_frame)
        exec ("transf.translation." + axis + " = p * step_size")
        transf.rotation.w = 1.0
        frames[p] = transf

        del transf

    return frames


def tf_to_matrix(transform):
    """Convert a geometry_msgs.Transform to a 3x4 numpy transformation matrix.
    
    :param transform: Transform to convert
    :type transform: geometry_msgs.Transform
    :return: 3x4 Transformation matrix
    :rtype: numpy.ndarray
    """
    trans_mat = tfs.translation_matrix([transform.translation.x,
                                        transform.translation.y,
                                        transform.translation.z])
    quat_mat = tfs.quaternion_matrix([transform.rotation.x,
                                      transform.rotation.y,
                                      transform.rotation.z,
                                      transform.rotation.w])

    return np.dot(trans_mat, quat_mat)


def matrix_to_tf(matrix):
    """Convert a 3x4 numpy transformation matrix to a geometry_msgs.Transform.
    
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
    tfs_ = None

    if type_ == 'linear':
        try:
            tfs_ = _create_linear_pattern(parent, *args)
        except ValueError, e:
            rospy.logwarn('Bad parameters: %s' % e)
    elif type_ == 'rectangular':
        try:
            tfs_ = _create_rectangular_pattern(parent, *args)
        except ValueError, e:
            rospy.logwarn('Bad parameters: %s' % e)
    elif type_ == 'circular':
        try:
            tfs_ = _create_circular_pattern(parent, *args)
        except ValueError, e:
            rospy.logwarn('Bad parameters: %s' % e)
    elif type_ == 'scatter':
        try:
            tfs_ = _create_scatter_pattern(parent, *args)
        except ValueError, e:
            rospy.logwarn('Bad parameters: %s' % e)
    else:
        pass

    return tfs_


def _create_linear_pattern(parent, num_points=0, step_size=0.0, line_len=0.0, axis='x'):

    try:
        (po, st, le) = handle_input_1d(num_points, step_size, line_len)
    except TypeError, e:
        rospy.logerr(e)

        return None

    pattern = tfs_along_axis(po, st, parent, axis=axis)

    return pattern.reshape(pattern.size)


def _create_rectangular_pattern(parent, num_points=(0, 0), step_sizes=(0, 0), line_lens=(0.0, 0.0)):

    try:
        (po_x, st_x, le_x) = handle_input_1d(num_points[0], step_sizes[0], line_lens[0])
        (po_y, st_y, le_y) = handle_input_1d(num_points[1], step_sizes[1], line_lens[1])
    except TypeError, e:
        rospy.logerr(e)

        return None

    rospy.logwarn(po_x)
    rospy.logwarn(type(po_x))

    rospy.logwarn(po_y)
    rospy.logwarn(type(po_y))

    pattern = np.array(np.empty([po_x, po_y]), dtype=XForm)
    x_pattern = tfs_along_axis(po_x, st_x, parent, axis='x')

    for i in range(po_x):
        y_pattern = tfs_along_axis(po_y, st_y, parent, basis_frame=x_pattern[i], axis='y')
        pattern[i, :] = y_pattern
        del y_pattern

    return pattern.reshape(pattern.size)


def _create_circular_pattern(parent, num_points=0, r=0.0, tan_rot=False, cw=False, angular_section=2*pi):

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

    pattern = np.array(np.empty(num_points), dtype=XForm)

    for i in range(num_points):
        t = XForm(parent)
        t.translation.x = r * cos(i * angular_resolution)
        t.translation.y = r * sin(i * angular_resolution)
        t.translation.z = 0.0

        if tan_rot:
            yaw = pi / 2 + i * angular_resolution
            M = pattern.tfs.euler_matrix(0, 0, yaw)
            q = matrix_to_tf(M).rotation
            t.rotation = q
        else:
            t.rotation.w = 1.0

        pattern[i] = t
        del t

    return pattern.reshape(pattern.size)


def _create_scatter_pattern(parent, points=None):
    points = []
    if type(points) == list:
        for p in points:
            if not type(p) == list:
                rospy.logerr("Single input point is not a list")
    else:
        rospy.logerr("Point input is not a list of points")

    if not len(points) > 0:
        rospy.logerr("Scatter point list is empty")

        return None

    pattern = np.array(np.empty(len(points)), dtype=XForm)

    i = 0
    for p in points:
        t = XForm(parent)

        if len(p) == 3:
            t.translation.x = p[0]
            t.translation.y = p[1]
            t.translation.z = p[2]
            t.rotation.w = 1.0
        elif len(p) == 6:
            t.translation.x = p[0]
            t.translation.y = p[1]
            t.translation.z = p[2]

            q = tfs.quaternion_from_euler(p[3], p[4], p[5], axes='sxyz')

            t.rotation.x = q[0]
            t.rotation.y = q[1]
            t.rotation.z = q[2]
            t.rotation.w = q[3]
        else:
            rospy.logerr("Incorrect point length (%s), aborting pattern generation" % len(p))

            return False

        pattern[i] = t
        i += 1

    return pattern.reshape(pattern.size)
