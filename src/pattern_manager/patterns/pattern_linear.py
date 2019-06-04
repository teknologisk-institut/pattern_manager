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

import pattern
import numpy as np


class PatternLinear(pattern.Pattern):
    """This class defines a linear pattern.
    """

    _alias_ = 'linear'

    def __init__(self, base_params, num_points=0, step_size=0, length=0, axis='x'):
        """The class constructor.
        
        :param base_params: Parameters for the base-clase (super).
        :type base_params: dict
        :param num_points: The number of points which the pattern consists of, defaults to 0
        :type num_points: int, optional
        :param step_size: The length between points in the pattern, defaults to 0
        :type step_size: int, optional
        :param length: The total length of the linear pattern, defaults to 0
        :type length: int, optional
        :param axis: The axis on which the pattern lies, defaults to 'x'
        :type axis: str, optional
        """

        super(PatternLinear, self).__init__(**base_params)

        try:
            self._input = handle_input_1d(num_points, step_size, length)
        except TypeError as error:
            print(error)

        if self._input is not False:
            self._points = self._input[0]
            self._step_size = self._input[1]
            self._length = self._input[2]
            self._axis = axis
            self._generate()

    def _generate(self):
        """This function generates the pattern from the values obtained in the constructor.
        
        :return: True if generation was successful.
        :rtype: bool
        """

        pattern = frames_along_axis(self._points, self._step_size, axis='x')
        self.finish_generation(pattern)

        return True