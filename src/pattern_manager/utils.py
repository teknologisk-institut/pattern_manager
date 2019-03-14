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
    frames = np.array(np.empty(count), dtype=gm.Transform)
    for p in range(count):
        transf = deepcopy(basis_frame)
        exec ("transf.translation." + axis + " = p * step_size")
        transf.rotation.w = 1.0
        frames[p] = transf
        del transf
    return frames


def transform_to_matrix(transform):
    trans_mat = tfs.translation_matrix([transform.translation.x,
                                        transform.translation.y,
                                        transform.translation.z])
    quat_mat = tfs.quaternion_matrix([transform.rotation.x,
                                      transform.rotation.y,
                                      transform.rotation.z,
                                      transform.rotation.w])
    return np.dot(trans_mat, quat_mat)


def matrix_to_transform(matrix):
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
