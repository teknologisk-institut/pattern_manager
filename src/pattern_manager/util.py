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

import geometry_msgs.msg as gm
import numpy as np
import tf.transformations as tfs


# logging output
import logging
logging.basicConfig(format='(Pattern Manager) %(levelname)s: %(message)s', level=logging.DEBUG)
output = logging


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
        output.error("1D - No parameters specified")

        return False
    elif 0 not in [number_of_points, step_size, line_length]:
        output.error("1D - Ambiguous parameters, all three specified (number_of_points, step_size, "
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


def frames_along_axis(count, step_size, basis_frame=gm.Transform(), axis='x'):
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
    frames = np.array(np.empty(count), dtype=gm.Transform)
    for p in range(count):
        transf = deepcopy(basis_frame)
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
    t = gm.Transform()
    t.translation.x = matrix[0, 3]
    t.translation.y = matrix[1, 3]
    t.translation.z = matrix[2, 3]

    q = tfs.quaternion_from_matrix(matrix)

    t.rotation.x = q[0]
    t.rotation.y = q[1]
    t.rotation.z = q[2]
    t.rotation.w = q[3]

    return t
