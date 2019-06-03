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

from pattern_manager.utils import output

import pattern
import geometry_msgs.msg as gm
import numpy as np
import tf.transformations as tfs


class PatternScatter(pattern.Pattern):
    """This class defines a scatter pattern.
    """

    _alias_ = 'scatter'

    def __init__(self, base_params, point_list):
        """The class constructor.
        
        :param base_params: Parameters for the base-class (super).
        :type base_params: dict
        :param point_list: A list of points of which the pattern consists.
        :type point_list: list
        """

        super(PatternScatter, self).__init__(**base_params)
        
        self.input_points = []
        if type(point_list) == list:
            for p in point_list:
                if type(p) == list:
                    self.input_points.append(p)
                else:
                    output.error("Single input point is not a list")
        else:
            output.error("Point input is not a list of points")

        if len(self.input_points) > 0:
            self._generate()
        else:
            output.error("Scatter point list is empty")

    def _generate(self):
        """This functions generates the pattern from the values obtained in the constructor.
        
        :return: True if generation was successful.
        :rtype: bool
        """

        pattern = np.array(np.empty(len(self.input_points)), dtype=gm.Transform)

        i = 0
        for p in self.input_points:
            t = gm.Transform()
            if len(p) == 3:
                # it's just a coordinate
                t.translation.x = p[0]
                t.translation.y = p[1]
                t.translation.z = p[2]
                t.rotation.w = 1.0
            elif len(p) == 6:
                # it's a transform
                t.translation.x = p[0]
                t.translation.y = p[1]
                t.translation.z = p[2]
                q = tfs.quaternion_from_euler(p[3],p[4],p[5],axes='sxyz')
                t.rotation.x = q[0]
                t.rotation.y = q[1]
                t.rotation.z = q[2]
                t.rotation.w = q[3]
            else:
                output.error("Incorrect point length (%s), aborting pattern generation" % len(p))
                return False
            pattern[i] = t
            i += 1

        self.finish_generation(pattern)

        return True