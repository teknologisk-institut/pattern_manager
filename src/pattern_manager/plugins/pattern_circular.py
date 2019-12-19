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
import tf.transformations as transformations

from pattern_manager.plugin import Plugin
from pattern_manager.util import matrix_to_tf
from pattern_manager.xform import XForm
from math import cos, sin, pi


class CircularPattern(Plugin):
    """
    This plugin class specifies the attributes of, and is responsible for the generation of, a circular XForm pattern

    :param parent: An XForm parent object under which to create the XForm pattern
    :type parent: XForm
    :param num_points: The number of points which make up the pattern
    :type num_points: int, optional
    :param r: The radius of the circular pattern
    :type r: float, optional
    :param tan_rot: Specifies whether to rotate each point tangent to the circle
    :type tan_rot: bool, optional
    :param cw: Specifies whether to spawn the circular pattern counter-clockwise
    :type cw: bool, optional
    :param angular_section: The size of an angular section
    :type angular_section: float, optional
    """

    def __init__(self, parent, num_points=0, r=0.0, tan_rot=False, cw=False, angular_section=2*pi):
        super(CircularPattern, self).__init__()

        self.parent = parent
        self.num_points = num_points
        self.r = r
        self.tan_rot = tan_rot
        self.cw = cw
        self.angular_section = angular_section

    def process(self):
        """
        This function generates the XForm pattern from the instance attributes
        """

        if abs(self.r) > 0:
            if self.num_points > 0:
                if self.angular_section == 0 or abs(self.angular_section - 2 * pi) < 0.01:
                    self.angular_section = 2 * pi
            else:
                rospy.logerr("Number of points is 0, cannot define a circular pattern")
        else:
            rospy.logerr("A radius of 0 is specified, cannot define a circular pattern")

        angular_resolution = self.angular_section / self.num_points

        if self.cw:
            angular_resolution *= -1

        if not self.angular_section == 2 * pi:
            self.num_points += 1

        xyz_set = set()

        for i in range(self.num_points):
            xyz_set.add((self.r * cos(i * angular_resolution), self.r * sin(i * angular_resolution), 0.0))

        tfs = []
        c = 0
        for xyz in xyz_set:
            tf = XForm(self.parent, name='{}_{}'.format(self.parent.name, c))
            tf.translation.x = xyz[0]
            tf.translation.y = xyz[1]
            tf.translation.z = xyz[2]

            if self.tan_rot:
                yaw = pi / 2 + c * angular_resolution
                m = transformations.euler_matrix(0, 0, yaw)
                q = matrix_to_tf(m).rotation
                tf.rotation = q
            else:
                tf.rotation.w = 1.0

            tfs.append(tf)

            c += 1

        return tfs
