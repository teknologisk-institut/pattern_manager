#!/usr/bin/env python
import pluginlib

from pattern_manager.patterns import Pattern
from pattern_manager.patterns import PatternLinear
from pattern_manager.patterns import PatternRectangular
from pattern_manager.patterns import PatternScatter
from pattern_manager.patterns import PatternCircular

import unittest


class TestPlugins(unittest.TestCase):
    def setUp(self):
        self._loader = pluginlib.PluginLoader(group='patterns')

    def test_pattern_plugin_keys_exist(self):
        for k in ['linear', 'rectangular', 'scatter', 'circular']:
            self.assertIn(k, self._loader.plugins['pattern'].keys())

    def test_pattern_plugins_exist(self):
        for k in self._loader.plugins:
            self.assertIsNotNone(self._loader.plugins[k])

    def test_pattern_plugin_types(self):
            l = self._loader.get_plugin('pattern', 'linear')
            r = self._loader.get_plugin('pattern', 'rectangular')
            s = self._loader.get_plugin('pattern', 'scatter')
            c = self._loader.get_plugin('pattern', 'circular')

            arr = [l, r, s, c]
            for p in arr:
                self.assertTrue(issubclass(p, Pattern))
            
            self.assertIs(l, PatternLinear)
            self.assertIs(r, PatternRectangular)
            self.assertIs(s, PatternScatter)
            self.assertIs(c, PatternCircular)


if __name__ == "__main__":
    unittest.main()
