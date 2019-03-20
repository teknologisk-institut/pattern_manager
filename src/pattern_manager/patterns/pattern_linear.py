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
from pattern_manager.utils import handle_input_1d, frames_along_axis

import pattern_base
import numpy as np


class PatternLinear(pattern_base.Pattern):
    _alias_ = 'linear'
    _parameterized = False

    def __init__(self, base_params, points=0, step_size=0, length=0, axis='x'):
        super(PatternLinear, self).__init__(**base_params)

        try:
            (p, s, l) = handle_input_1d(points, step_size, length)
            self._parameterized = True
            self.points = p
            self.step_size = s
            self.length = l
            self.axis = axis
        except TypeError as error:
            print(error)

    def generate_pattern(self):
        if not self.can_generate():
            return False

        self._pattern = frames_along_axis(self.points,
                                          self.step_size,
                                          axis=self.axis)
        self.finish_generation()
        self._pattern_org_copy = np.copy(self._pattern)

        return True
