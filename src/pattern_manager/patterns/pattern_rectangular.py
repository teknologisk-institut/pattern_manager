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

from .. import utils
from pattern_manager.utils import frames_along_axis

import pattern_base
import numpy as np
import geometry_msgs.msg as gm


class PatternRectangular(pattern_base.Pattern):
    _alias_ = 'rectangular'
    _parameterized = False

    def __init__(self, base_params, points=(0, 0), step_sizes=(0.0, 0.0), lengths=(0, 0)):
        super(PatternRectangular, self).__init__(**base_params)

        try:
            (px, sx, lx) = utils.handle_input_1d(points[0], step_sizes[0], lengths[0])
            (py, sy, ly) = utils.handle_input_1d(points[1], step_sizes[1], lengths[1])
            self.points = (px, py)
            self.step_sizes = (sx, sy)
            self.lengths = (lx, ly)
            self._parameterized = True
        except TypeError as error:
            print(error)

    def generate_pattern(self):
        if not self.can_generate():
            return False

        # pattern is x-major, numpy is row-major
        self._pattern = np.array(np.empty((self.points[0], self.points[1])),
                                 dtype=gm.Transform)

        x_pattern = frames_along_axis(self.points[0],
                                      self.step_sizes[0],
                                      axis='x')

        for x in range(self.points[0]):
            y_pattern = frames_along_axis(self.points[1],
                                          self.step_sizes[1],
                                          basis_frame=x_pattern[x],
                                          axis='y')
            self._pattern[x, :] = y_pattern
            del y_pattern

        self.finish_generation()
        self._pattern_org_copy = np.copy(self._pattern)

        return True
