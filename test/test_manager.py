#!/usr/bin/env python

from pattern_manager.collection import Manager
from pattern_manager.collection import Group, GType
from pattern_manager.patterns import PatternLinear, PatternRectangular
from pattern_manager.examples import example_dicts as ex

import unittest


class TestManager(unittest.TestCase):
    def setUp(self):
        pass

    def test_register_id_func(self):
        o = object
        Manager.register_id(id(object))

        self.assertIn(id(o), Manager.i.keys())
        self.assertIn(id(o), Manager.finished.keys())
        self.assertIn(id(o), Manager.active.keys())

        self.assertTrue(Manager.i[id(o)] == 0)
        self.assertTrue(Manager.finished[id(o)] == False)
        self.assertTrue(Manager.active[id(o)] == False)

    def test_iterate_func(self):
        g0 = Group(GType.GOG, "g0")
        g1 = Group(GType.GOP, "g1", par=g0)
        g2 = Group(GType.GOG, "g2", par=g0)

        p1 = PatternLinear(ex.linear_d['base_params'], **ex.linear_d['pattern_params'])
        p2 = PatternRectangular(ex.rect_d['base_params'], **ex.rect_d['pattern_params'])

        g1.add_node(p1)
        g1.add_node(p2)

        self.assertTrue(g0.chldrn[Manager.i[id(g0)]] == g1)
        self.assertTrue(g1.chldrn[Manager.i[id(g1)]] == p1)
        
        Manager.iterate(g1)

        self.assertTrue(g1.chldrn[Manager.i[id(g1)]] == p2)
       
        Manager.iterate(g1)

        self.assertTrue(g0.chldrn[Manager.i[id(g0)]] == g2)


if __name__ == '__main__':
    unittest.main()
