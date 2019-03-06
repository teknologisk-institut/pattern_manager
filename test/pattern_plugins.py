#!/usr/bin/env python
import pluginlib
from pattern_manager.patterns import pattern_base

import unittest


class TestPlugins(unittest.TestCase):
    def test_pattern_plugins(self):
        try:
            loader = pluginlib.PluginLoader()
            plugins = loader.plugins
            print(plugins['pattern']['linear'])
            print(plugins['pattern']['rectangular'])
            print(plugins['pattern']['scatter'])

            return True
        except AttributeError as e:
            self.fail("Could not load plugins (%s)" % e)


if __name__ == "__main__":
    test_runner = unittest.TextTestRunner()
    unittest.main(testRunner=test_runner)
