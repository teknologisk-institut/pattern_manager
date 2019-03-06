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


class PatternLinear(pattern_base.Pattern):
    _alias_ = 'linear'

    """ Points along x-axis of origin frame by default """
    _points = 0
    _step_size = 0
    _length = 0
    _axis = 'x'

    def set_pattern_parameters(self, number_of_points=0, step_size=0, line_length=0, axis='x'):
        try:
            (po, st, le) = utils.handle_input_1d(number_of_points, step_size, line_length)
        except TypeError:
            return False
        self._step_size = st
        self._points = po
        self._length = le
        self._axis = axis
        self.set_parameterized(True)
        return True

    def generate_pattern(self):
        if not self.can_generate():
            return False
        self._pattern = utils.frames_along_axis(self._points,
                                          self._step_size,
                                          axis=self._axis)
        self._pattern_org_copy = np.copy(self._pattern)
        self.finish_generation()
        self._pattern_org_copy = np.copy(self._pattern)
        return True
