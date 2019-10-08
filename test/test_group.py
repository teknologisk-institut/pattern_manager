#!/usr/bin/env python

from pattern_manager.collection import GType, Group
from pattern_manager.patterns import PatternLinear
from pattern_manager.examples import example_dicts as ex

import unittest


class TestGroup(unittest.TestCase):
    def setUp(self):
        self.g1 = Group(GType.GOG, "g1")
        self.g2 = Group(GType.GOP, "g2", self.g1)

    def test_group_type(self):
        self.assertTrue(self.g1.g_typ == "GOG")
        self.assertTrue(self.g2.g_typ == "GOP")

    def test_group_parent(self):
        self.assertTrue(self.g2.par == self.g1)

    def test_add_child_func(self):
        g1 = Group(GType.GOG, "g1")
        g2 = Group(GType.GOP, "g2", par=g1)

        p1 = PatternLinear(ex.linear_d['base_params'], **ex.linear_d['pattern_params'])
        g2.add_node(p1)

        self.assertTrue(Group.get_sub_by_name("g2", g1) == g2)
        self.assertTrue(Group.get_sub_by_name("cheese_linear", g1) == p1)

    def test_chld_cnt(self):
        self.assertTrue(len(self.g1.chldrn) == 1)
        self.assertTrue(len(self.g2.chldrn) == 0)

    def test_get_sub_by_nm_func(self):
        self.assertTrue(Group.get_sub_by_name("g2", self.g1) == self.g2)


if __name__ == '__main__':
    unittest.main()