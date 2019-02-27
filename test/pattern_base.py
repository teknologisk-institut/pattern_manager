#!/usr/bin/env python3

import unittest


class TestImport(unittest.TestCase):

    def test_import_patterns(self):
        try:
            import pattern_manager.pattern
            return True
        except ImportError:
            self.fail("Could not import the base patterns module")


if __name__ == "__main__":
    test_runner = unittest.TextTestRunner()
    unittest.main(testRunner=test_runner)