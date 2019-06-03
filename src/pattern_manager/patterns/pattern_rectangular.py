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

from pattern_manager.utils import frames_along_axis, handle_input_1d

import pattern
import numpy as np
import geometry_msgs.msg as gm


class PatternRectangular(pattern.Pattern):
    """This class defines a rectangular pattern.
    """

    _alias_ = 'rectangular'

    def __init__(self, base_params, num_points=(0, 0), step_sizes=(0.0, 0.0), lengths=(0.0, 0.0)):
        """The class constructor.
        
        :param base_params: Parameters for the base-class (super).
        :type base_params: dict
        :param num_points: The number of points of which the pattern consists, defaults to (0, 0)
        :type num_points: tuple, optional
        :param step_sizes: The lengths between points on two axiis, defaults to (0.0, 0.0)
        :type step_sizes: tuple, optional
        :param lengths: The lengths of the rectangles two axiis, defaults to (0.0, 0.0)
        :type lengths: tuple, optional
        """

        super(PatternRectangular, self).__init__(**base_params)

        try:
            self.inputX = handle_input_1d(num_points[0], step_sizes[0], lengths[0])
            self.inputY = handle_input_1d(num_points[1], step_sizes[1], lengths[1])
        except TypeError as error:
            print(error)

        if self.inputX is not False and self.inputY is not False:
            self.points = (self.inputX[0], self.inputY[0])
            self.step_size = (self.inputX[1], self.inputY[1])
            self.length = (self.inputX[2], self.inputY[2])
            self._generate()

    def _generate(self):
        """This functions generates the pattern from the values obtained in the constructor.
        
        :return: True if generation was successful.
        :rtype: bool
        """

        pattern = np.array(np.empty((self.points[0], self.points[1])), dtype=gm.Transform)
        x_pattern = frames_along_axis(self.points[0], self.step_size[0], axis='x')

        for x in range(self.points[0]):
            y_pattern = frames_along_axis(self.points[1], self.step_size[1], basis_frame=x_pattern[x], axis='y')
            pattern[x, :] = y_pattern
            del y_pattern

        self.finish_generation(pattern)

        return True