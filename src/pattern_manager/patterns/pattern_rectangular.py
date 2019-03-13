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

import pattern_base
import numpy as np


class PatternRectangular(pattern_base.Pattern):
    _alias_ = 'rectangular'

    _points = (0, 0)
    _step_size = (0.0, 0.0)
    _length = (0.0, 0.0)

    def set_pattern_parameters(self, points_x=0, points_y=0, step_x=0, step_y=0, len_x=0, len_y=0):
        try:
            (px, sx, lx) = utils.handle_input_1d(points_x, step_x, len_x)
            (py, sy, ly) = utils.handle_input_1d(points_y, step_y, len_y)
        except TypeError:
            return False
        self._points = (px, py)
        self._step_size = (sx, sy)
        self._length = (lx, ly)
        self.parameterized = True
        return True

    def generate_pattern(self):
        if not self.can_generate():
            return False
        # pattern is x-major, numpy is row-major
        self._pattern = np.array(np.empty((self._points[0], self._points[1])), dtype=gm.Transform)
        #print self._pattern
        x_pattern = utils.frames_along_axis(self._points[0],
                                      self._step_size[0],
                                      axis='x')
        for x in range(self._points[0]):
            y_pattern = utils.frames_along_axis(self._points[1],
                                          self._step_size[1],
                                          basis_frame=x_pattern[x],
                                          axis='y')
            self._pattern[x,:] = y_pattern
            del y_pattern
        self.finish_generation()
        self._pattern_org_copy = np.copy(self._pattern)
        return True


def main():
    print('I\'m a linear plugin')


if __name__ == '__main__':
    main()
