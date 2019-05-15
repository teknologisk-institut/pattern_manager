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
    """A circular pattern, with positions along the perimeter.
    
    Pattern is defined by a number of positions along the perimeter of a circle with specified radius. 
    All positions have the same orientation as the parent frame, and listed in counter-clockwise order.

    Optional parameters allow:
    - only use a section of a circle (an arc) (rather than the whole circle)
    - clockwise iteration (rather than counter-clockwise)
    - individual rotation following the tangent of the circle (rather than maintaining the same orientation)
    - any combination of the above
    """
    _alias_ = 'circular'

    def __init__(self, base_params, r=0.0, num_points=0, tan_rot=False, cw=False, angular_section=0.0):
        """Initialize pattern.
        
        :param r: radius of the circular pattern  
        :type r: float
        :param num_points: Number of points along the circle
        :type num_points: int
        :param tan_rot: Should the x-axis follow the tangent of the circle, defaults to False
        :type tan_rot: bool, optional
        :param cw: List the positions going clockwise around the circle center, defaults to False
        :type cw: bool, optional
        :param angular_section: Angular section to define the pattern for in rad, defaults to 2*pi
        :type angular_section: float, optional
        """
        super(PatternCircular, self).__init__(**base_params)

        if abs(r) > 0:
            if num_points > 0:
                self._radius = r
                self._num_points = num_points
                self._tan_rot = tan_rot

                if angular_section == 0 or abs(angular_section - 2 * pi) < 0.01:
                    self._ang_sec = 2 * pi
                else:
                    self._ang_sec = angular_section
                    utils.output.debug("Creating an angular section of %srad" % self._ang_sec)

                self._cw = cw
                if self._cw:
                    utils.output.debug("Clockwise rotation specified")

                if self._tan_rot:
                    utils.output.debug("Rotation will follow tangent of circle")

                self._generate_pattern()
            else:
                utils.output.error("Number of points is 0, can't define this circular pattern")
        else:
            utils.output.error("A radius of 0 is specified, can't define this circular pattern")

    def _generate_pattern(self):
        angular_resolution = self._ang_sec / self._num_points

        if self._cw:
            angular_resolution *= -1

        if not self._ang_sec == 2 * pi:
            self._num_points += 1

        pattern = np.array(np.empty(self._num_points), dtype=gm.Transform)
            
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
                
            pattern[i] = t
            del t

        self.finish_generation()

        for f in pattern:
            self.add_element(f)

        return True
