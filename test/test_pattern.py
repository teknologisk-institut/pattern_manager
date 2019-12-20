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

import unittest
import numpy as np

from pattern_manager.xform import XForm
from pattern_manager.plugins import \
    pattern_linear, \
    pattern_rectangular, \
    pattern_scatter, \
    pattern_circular


class PatternTestCase(unittest.TestCase):

    def setUp(self):
        self.root = XForm(None, 'root')

    def test_linear_pattern(self):
        pattern = pattern_linear.LinearPattern(self.root, num_points=4, step_size=1)
        xforms = pattern.process()

        self.assertEquals(len(self.root.get_nodes(self.root)), 5)

        x = [0.0, 1.0, 2.0, 3.0]

        for xf in xforms:
            self.assertIn(xf.translation.x, x)

            x.remove(xf.translation.x)

    def test_rectangular_pattern(self):
        pattern = pattern_rectangular.RectangularPattern(self.root, num_points=(2, 2), step_sizes=(1, 1))
        xforms = pattern.process()

        self.assertEquals(len(self.root.get_nodes(self.root)), 5)

        x = [0.0, 1.0, 0.0, 1.0]
        y = [0.0, 1.0, 0.0, 1.0]

        for xf in xforms:
            self.assertIn(xf.translation.x, x)
            self.assertIn(xf.translation.y, y)

            x.remove(xf.translation.x)
            y.remove(xf.translation.y)

    def test_circular_pattern(self):
        pattern = pattern_circular.CircularPattern(self.root, num_points=4, r=1)
        xforms = pattern.process()

        self.assertEquals(len(self.root.get_nodes()), 5)

        x = [0.0, 0.0, 1.0, -1.0]
        y = [0.0, 0.0, 1.0, -1.0]

        for xf in xforms:
            for f in x:

                if np.isclose(xf.translation.x, f, rtol=1e-05, atol=1e-08, equal_nan=False):
                    x.remove(f)

            for f in y:

                if np.isclose(xf.translation.y, f, rtol=1e-05, atol=1e-08, equal_nan=False):
                    y.remove(f)

        self.assertEquals(len(x), 0)
        self.assertEquals(len(y), 0)

    def test_scatter_pattern(self):
        pattern = pattern_scatter.ScatterPattern(self.root, points=[
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0)
        ])
        xforms = pattern.process()

        self.assertEquals(len(self.root.get_nodes(self.root)), 4)

        x = [0.0, 0.0, 1.0]
        y = [0.0, 0.0, 1.0]
        z = [0.0, 0.0, 1.0]

        for xf in xforms:
            self.assertIn(xf.translation.x, x)
            self.assertIn(xf.translation.y, y)
            self.assertIn(xf.translation.z, z)

            x.remove(xf.translation.x)
            y.remove(xf.translation.y)
            z.remove(xf.translation.z)

    def tearDown(self):
        del self.root


if __name__ == '__main__':
    unittest.main()
