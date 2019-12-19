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

from pattern_manager import XForm


class XFormTestCase(unittest.TestCase):

    def setUp(self):
        self.root = XForm(None, 'root')

    def test_add_node(self):
        chld1 = XForm(self.root, 'chld1')
        chld2 = XForm(self.root, 'chld2')

        chld1.add_node(chld2)

        self.assertIn(id(chld2), chld1.children.keys())
        self.assertNotIn(id(chld2), self.root.children.keys())
        self.assertEquals(id(chld1), id(chld2.parent))

        del chld1
        del chld2

    def test_set_active(self):
        chld1 = XForm(self.root, 'chld1')

        chld1.set_active(True)
        self.assertTrue(chld1.active)

        chld1.set_active(False)
        self.assertFalse(chld1.active)

        chld2 = XForm(chld1, 'chld2')
        chld3 = XForm(chld1, 'chld3')

        chld1.set_active(True)

        self.assertFalse(chld1.active)
        self.assertTrue(chld2.active)
        self.assertTrue(chld3.active)

        chld1.set_active(False)

        self.assertFalse(chld1.active)
        self.assertFalse(chld2.active)
        self.assertFalse(chld3.active)

        del chld1
        del chld2
        del chld3

    def test_get_current_node(self):
        chld1 = XForm(self.root, 'chld1')
        chld2 = XForm(chld1, 'chld2')
        chld3 = XForm(chld1, 'chld3')

        self.root.set_active(True)

        self.assertEquals(id(self.root.get_current_node()), id(chld2))

        del chld1
        del chld2
        del chld3

    def test_get_active_nodes(self):
        chld1 = XForm(self.root, 'chld1')
        chld2 = XForm(chld1, 'chld2')
        chld3 = XForm(chld1, 'chld3')

        self.root.set_active(True)

        actv_ids = self.ids_from_nodes(self.root.get_active_nodes())

        self.assertEquals(len(actv_ids), 2)
        self.assertIn(id(chld2), actv_ids)
        self.assertIn(id(chld3), actv_ids)

        del chld1
        del chld2
        del chld3

    def test_recursive_remove_node(self):
        chld1 = XForm(self.root, 'chld1')
        chld2 = XForm(chld1, 'chld2')
        chld3 = XForm(chld1, 'chld3')

        node_ids = self.ids_from_nodes(self.root.get_nodes())

        self.assertEquals(len(node_ids), 4)
        self.assertIn(id(self.root), node_ids)
        self.assertIn(id(chld1), node_ids)
        self.assertIn(id(chld2), node_ids)
        self.assertIn(id(chld3), node_ids)

        self.root.recursive_remove_node(id(chld1))

        node_ids = self.ids_from_nodes(self.root.get_nodes())

        self.assertEquals(len(node_ids), 1)
        self.assertIn(id(self.root), node_ids)

        del chld1
        del chld2
        del chld3

    def test_get_nodes(self):
        chld1 = XForm(self.root, 'chld1')

        ids = self.ids_from_nodes(self.root.get_nodes())

        self.assertEquals(len(ids), 2)
        self.assertIn(id(self.root), ids)
        self.assertIn(id(chld1), ids)

        chld2 = XForm(chld1, 'chld2')
        chld3 = XForm(chld2, 'chld3')
        chld4 = XForm(chld2, 'chld4')

        ids = self.ids_from_nodes(self.root.get_nodes())

        self.assertEquals(len(ids), 5)
        self.assertIn(id(self.root), ids)
        self.assertIn(id(chld1), ids)
        self.assertIn(id(chld2), ids)
        self.assertIn(id(chld3), ids)
        self.assertIn(id(chld4), ids)

        self.root.recursive_remove_node(id(chld2))

        ids = self.ids_from_nodes(self.root.get_nodes())

        self.assertEquals(len(ids), 2)
        self.assertIn(id(self.root), ids)
        self.assertIn(id(chld1), ids)

        del chld1
        del chld2
        del chld3
        del chld4

    def test_get_node(self):
        chld = XForm(self.root, 'chld')

        self.assertEquals(id(self.root), id(self.root.get_node(id(self.root))))
        self.assertEquals(id(chld), id(self.root.get_node(id(chld))))

        del chld

    def test_iterate(self):
        chld1 = XForm(self.root, 'chld1')
        chld2 = XForm(chld1, 'chld2')
        chld3 = XForm(chld2, 'chld3')
        chld4 = XForm(chld2, 'chld4')
        chld5 = XForm(chld2, 'chld5')

        self.root.set_active(True)

        self.assertEquals(id(self.root.get_current_node()), id(chld3))

        self.root.iterate()

        self.assertEquals(id(self.root.get_current_node()), id(chld4))

        self.root.iterate()

        self.assertEquals(id(self.root.get_current_node()), id(chld5))

        del chld1
        del chld2
        del chld3
        del chld4
        del chld5

    def test_to_dict(self):
        chld1 = XForm(self.root, 'chld1')
        chld2 = XForm(chld1, 'chld2')
        chld3 = XForm(chld1, 'chld3')

        xform_dict = self.root.to_dict()
        new_dict = {
            "root": {
                "chld1": {
                    "translation": [
                        0.0,
                        0.0,
                        0.0
                    ],
                    "ref_frame": "root",
                    "rotation": [
                        0.0,
                        0.0,
                        0.0,
                        1.0
                    ],
                    "chld2": {
                        "translation": [
                            0.0,
                            0.0,
                            0.0
                        ],
                        "ref_frame": "chld1",
                        "rotation": [
                            0.0,
                            0.0,
                            0.0,
                            1.0
                        ]
                    },
                    "chld3": {
                        "translation": [
                            0.0,
                            0.0,
                            0.0
                        ],
                        "ref_frame": "chld1",
                        "rotation": [
                            0.0,
                            0.0,
                            0.0,
                            1.0
                        ]
                    }
                },
                "translation": [
                    0.0,
                    0.0,
                    0.0
                ],
                "ref_frame": None,
                "rotation": [
                    0.0,
                    0.0,
                    0.0,
                    1.0
                ]
            }
        }

        self.assertDictEqual(xform_dict, new_dict)

        del chld1
        del chld2
        del chld3

    def test_from_dict(self):
        xform_dict = {
            "root": {
                "chld1": {
                    "translation": [
                        0.0,
                        0.0,
                        0.0
                    ],
                    "ref_frame": "root",
                    "rotation": [
                        0.0,
                        0.0,
                        0.0,
                        1.0
                    ],
                    "chld2": {
                        "translation": [
                            0.0,
                            0.0,
                            0.0
                        ],
                        "ref_frame": "chld1",
                        "rotation": [
                            0.0,
                            0.0,
                            0.0,
                            1.0
                        ]
                    },
                    "chld3": {
                        "translation": [
                            0.0,
                            0.0,
                            0.0
                        ],
                        "ref_frame": "chld1",
                        "rotation": [
                            0.0,
                            0.0,
                            0.0,
                            1.0
                        ]
                    }
                },
                "translation": [
                    0.0,
                    0.0,
                    0.0
                ],
                "ref_frame": None,
                "rotation": [
                    0.0,
                    0.0,
                    0.0,
                    1.0
                ]
            }
        }

        self.root.from_dict(xform_dict)

        self.assertDictEqual(self.root.to_dict(), xform_dict)

        self.root.recursive_remove_node(id(self.root))

    @staticmethod
    def ids_from_nodes(nodes):
        ids = []
        for node in nodes:
            ids.append(id(node))

        return ids


if __name__ == '__main__':
    unittest.main()
