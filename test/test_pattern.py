import unittest

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
        pass
        # parent = XForm(None, 'root')
        # pattern = pattern_circular.CircularPattern(parent, num_points=4, r=1)
        # xforms = pattern.process()
        #
        # self.assertEquals(len(XForm.get_nodes()), 4)
        #
        # x = [0.0, 1.0, 0.0, 1.0]
        # y = [0.0, 1.0, 0.0, 1.0]
        #
        # for xf in xforms:
        #     self.assertIn(xf.translation.x, x)
        #     self.assertIn(xf.translation.y, y)
        #
        #     x.remove(xf.translation.x)
        #     y.remove(xf.translation.y)
        #
        # XForm.recursive_remove_node(id(XForm.root))

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
