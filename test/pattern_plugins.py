#!/usr/bin/env python
import pluginlib

from pattern_manager.patterns import PatternLinear
from pattern_manager.patterns import PatternRectangular
from pattern_manager.patterns import PatternScatter

import unittest

loader = pluginlib.PluginLoader(group='patterns')


class TestPlugins(unittest.TestCase):
    def test_pattern_plugin_keys_exist(self):
        p = ['linear', 'rectangular', 'scatter']
        for k in p:
            self.assertIn(k, loader.plugins['pattern'].keys())

    def test_pattern_plugins_exist(self):
        for k in loader.plugins:
            self.assertIsNotNone(loader.plugins[k])

    def test_pattern_plugin_types(self):
            l = loader.get_plugin('pattern', 'linear')()
            r = loader.get_plugin('pattern', 'rectangular')()
            s = loader.get_plugin('pattern', 'scatter')()

            self.assertIsInstance(l, PatternLinear)
            self.assertIsInstance(r, PatternRectangular)
            self.assertIsInstance(s, PatternScatter)


if __name__ == "__main__":
    test_runner = unittest.TextTestRunner()
    unittest.main(testRunner=test_runner)
