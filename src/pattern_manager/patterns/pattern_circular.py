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

# Author: Mikkel Rath Hansen

from __future__ import division
from .. import utils

import pattern_base
import numpy as np
import geometry_msgs.msg as gm

from math import cos, sin, pi


class PatternCircular(pattern_base.Pattern):
    _alias_ = 'circular'

    def set_pattern_parameters(self, radius=0.0, number_of_points=0, tangent_rotation=False, clockwise=False, angular_section=0.0):
        if radius == 0.0:
            utils.output.error("A radius of 0 is specified, can't define this circular pattern")
            return False
        if number_of_points == 0:
            utils.output.error("Number of points is 0, can't define this circular pattern")
            return False
        self._radius = radius
        self._num_points = number_of_points
        self._tan_rot = tangent_rotation
        if angular_section == 0 or abs(angular_section - 2 * pi) < 0.01:
            self._ang_sec = 2 * pi
        else:
            self._ang_sec = angular_section
            utils.output.debug("Creating an angular section of %srad" % self._ang_sec)
        self._cw = clockwise
        if self._cw:
            utils.output.debug("Clockwise rotation specified")
        if self._tan_rot:
            utils.output.debug("Rotation will follow tangent of circle")
        self.parameterized = True
        return True

    def generate_pattern(self):
        angular_resolution = self._ang_sec / self._num_points
        if self._cw:
            angular_resolution *= -1
        if not self._ang_sec == 2 * pi:
            self._num_points += 1
        self._pattern = np.array(np.empty(self._num_points), dtype=gm.Transform)
        for i in range(self._num_points):
            t = gm.Transform()
            t.translation.x = self._radius * cos(i * angular_resolution)
            t.translation.y = self._radius * sin(i * angular_resolution)
            t.translation.z = 0.0
            if self._tan_rot:
                yaw = pi / 2 + i * angular_resolution
                M = pattern_base.tfs.euler_matrix(0, 0, yaw)
                q = utils.matrix_to_transform(M).rotation
                t.rotation = q
            else:
                t.rotation.w = 1.0
            self._pattern[i] = t
            del t
        self._pattern_org_copy = np.copy(self._pattern)
        self.finish_generation()
        return True
