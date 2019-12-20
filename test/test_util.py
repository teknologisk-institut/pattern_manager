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

import math
import unittest
import tf.transformations as transformations
import pattern_manager.util as util


class UtilTestCase(unittest.TestCase):

    def setUp(self):
        pass

    def test_handle_input_1d(self):
        self.assertFalse(util.handle_input_1d(0, 0.0, 0.0))
        self.assertFalse(util.handle_input_1d(2, 1.0, 2.0))
        self.assertEquals(util.handle_input_1d(3, 1.0, 0.0), (3, 1.0, 2.0))
        self.assertEquals(util.handle_input_1d(3, 0.0, 2.0), (3, 1.0, 2.0))
        self.assertEquals(util.handle_input_1d(0, 1.0, 2.0), (3, 1.0, 2.0))

    def test_matrix_to_tf(self):
        m = transformations.euler_matrix(0, 0, 1)
        rot = util.matrix_to_tf(m).rotation
        q = [rot.x, rot.y, rot.z, rot.w]

        expected = [0.0, 0.0, 0.479425538604, 0.87758256189]
        for i in range(len(expected)):
            self.assertAlmostEquals(expected[i], q[i])
