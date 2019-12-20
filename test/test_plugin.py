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

from pattern_manager import PluginLoader, Plugin
from pattern_manager.plugins import \
    pattern_linear, \
    pattern_rectangular, \
    pattern_scatter, \
    pattern_circular


class PluginTestCase(unittest.TestCase):

    def setUp(self):
        self.loader = PluginLoader('pattern_manager.plugins')

    def test_load_plugins(self):
        plugins = [
            pattern_linear.LinearPattern,
            pattern_rectangular.RectangularPattern,
            pattern_circular.CircularPattern,
            pattern_scatter.ScatterPattern
        ]

        for plugin in plugins:
            self.assertIn(plugin, self.loader.plugins.values())

    def test_type(self):

        for plugin in self.loader.plugins.values():
            assert issubclass(plugin, Plugin)

    def tearDown(self):
        del self.loader


if __name__ == '__main__':
    unittest.main()
