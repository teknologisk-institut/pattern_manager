#!/usr/bin/env python

import unittest


class TestImport(unittest.TestCase):

    def test_import_patterns(self):
        try:
            import base_pattern.pattern
            return True
        except ImportError as e:
            self.fail("Could not import the base patterns module (%s)" % e)


if __name__ == "__main__":

    test_runner = unittest.TextTestRunner()
    unittest.main(testRunner=test_runner)