#!/usr/bin/env python

import unittest

class TestBase(unittest.TestCase):

    def test_import_patterns(self):
        try:
            import pattern_manager.patterns.pattern_base
            return True
        except ImportError as e:
            self.fail("Could not import the base patterns module (%s)" % e)


class TestCircular(unittest.TestCase):

    radius = 0.5
    points = 4

    def setup_and_generate(self, **kwargs):
        from pattern_manager.patterns.pattern_circular import PatternCircular
        p = PatternCircular()
        p.set_pattern_parameters(radius=self.radius, number_of_points=self.points, **kwargs)
        p.generate_pattern()
        return p

    def test_parameters(self):
        from pattern_manager.patterns.pattern_circular import PatternCircular
        p = PatternCircular()
        ret = p.set_pattern_parameters(radius=0.5, number_of_points=10)
        self.assertTrue(ret)
        self.assertTrue(p.parameterized)
        ret = p.set_pattern_parameters(radius=0.5, number_of_points=0)
        self.assertFalse(ret)
        ret = p.set_pattern_parameters(radius=0, number_of_points=10)
        self.assertFalse(ret)

    def test_generated_pattern(self):
        p = self.setup_and_generate()
        # test first position, always at (r,0)
        self.assertAlmostEqual(p._pattern[0].translation.x, self.radius)
        self.assertAlmostEqual(p._pattern[0].translation.y, 0.0)
        # test second position, at (0,r)
        self.assertAlmostEqual(p._pattern[1].translation.x, 0.0)
        self.assertAlmostEqual(p._pattern[1].translation.y, self.radius)
        
    def test_option_tangent(self):
        p = self.setup_and_generate(tangent_rotation=True)
        #print p._pattern
        self.assertAlmostEqual(p._pattern[0].rotation.w, p._pattern[0].rotation.z)
        self.assertAlmostEqual(p._pattern[1].rotation.z, 1.0)

    def test_option_clockwise(self):
        p_cw = self.setup_and_generate(clockwise=True)
        p_ccw = self.setup_and_generate(clockwise=False)
        self.assertEqual(p_ccw._pattern[0], p_cw._pattern[0])
        self.assertNotEqual(p_ccw._pattern[1], p_cw._pattern[1])
        self.assertAlmostEqual(p_ccw._pattern[1].translation.y, p_cw._pattern[3].translation.y)
        self.assertAlmostEqual(p_ccw._pattern[1].translation.x, p_cw._pattern[3].translation.x)

    def test_option_angular_section(self):
        from math import pi, cos
        sec = 0.666
        p = self.setup_and_generate(angular_section=sec)
        self.assertEqual(p._pattern[-1].translation.x, self.radius * cos(sec))


if __name__ == "__main__":

    test_runner = unittest.TextTestRunner()
    unittest.main(testRunner=test_runner)