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

# Author: Mikkel Rath Hansen

from pluginlib import PluginLoader
from pattern_manager.patterns import pattern_base
from pattern_manager.collection import Manager
from copy import copy

""" Factory for creating the specified pattern object """
class PatternFactory:
    def __init__(self):
        self._patterns = {}

    def register_pattern_type(self, pattern_type, pattern):
        self._patterns[pattern_type] = pattern

    def get_pattern(self, pattern_type, base_params, pattern_params):
        pattern = self._patterns.get(pattern_type)

        if not pattern:
            raise ValueError(pattern_type)

        return pattern(base_params, **pattern_params)


""" This singleton is the interface of the pattern manager """
class Interface(object):
    _instance = None

    @staticmethod
    def getInstance():
        if Interface._instance == None:
            Interface()

        return Interface._instance

    def __init__(self, pattern_dicts=[]):
        if Interface._instance != None:
            raise Exception("Instance already exists!")
        else:
            Interface._instance = self

        self.factory = PatternFactory()
        self.loader = PluginLoader(group='patterns')
        self._load_pattern_types()
        self.patterns = {}
        self.groups = {}
        self.pattern_id = 0
        self.group_id = 0

        for d in pattern_dicts:
            self.create_pattern_from_dict(
                d['pattern_params'],
                d['base_params']
            )

    """ Loads the different pattern types from plugin classes """
    def _load_pattern_types(self):
        for k in self.loader.plugins['pattern'].keys():
            self.factory.register_pattern_type(k, self.loader.get_plugin('pattern', k))

    """ Creates a pattern from a dictionary specifying the various pattern parameters """
    def create_pattern_from_dict(self, pattern_params, base_params):
        pattern_type = pattern_params.pop('pattern_type')
        pattern = self.factory.get_pattern(pattern_type, base_params, pattern_params)
        self.patterns[self.pattern_id] = pattern
        self.pattern_id += 1

        return pattern

    """ Group together objects of type Manager or Pattern """
    def group(self, elements):
        id = copy(self.group_id)
        manager = Manager(elements)
        self.groups[id] = manager
        self.group_id += 1

        return id

    """ Removes the group from the groups dictionary """
    def ungroup(self, id):
        try:
            del self.groups[id]
        except KeyError:
            return False


def intf_example():
    linear_d = {
        'pattern_params': {
            'pattern_type': 'linear',
            'step_size': 0.1,
            'num_points': 3
        },
        'base_params': {
            'i': 0,
            'name': 'cheese_linear',
            'rev': False,
            'frame': 'base_link',
            'offset_xy': [0.5, 0.3],
            'offset_rot': 0.2,
        }
    }

    linear_d2 = {
        'pattern_params': {
            'pattern_type': 'linear',
            'step_size': 0.1,
            'num_points': 3
        },
        'base_params': {
            'i': 0,
            'name': 'cheese_linear',
            'rev': False,
            'frame': 'base_link',
            'offset_xy': [0.5, 0.3],
            'offset_rot': 0.2,
        }
    }

    rect_d = {
        'pattern_params': {
            'pattern_type': 'rectangular',
            'num_points': (3, 2),
            'step_sizes': (0.1, 0.1),
        },
        'base_params': {
            'i': 0,
            'name': 'cheese_rect',
            'rev': False,
            'frame': 'base_link',
        }
    }

    scatter_d = {
        'pattern_params': {
            'pattern_type': 'scatter',
            'point_list':
                [
                    [1.0, 1.0, 1.0],
                    [1.1, 1.0, 1.0],
                    [1.2, 1.0, 1.0],
                    [1.0, 1.0, 0.5, 0.0, 0.0, 0.5],
                    [1.1, 1.0, 0.5, 0.0, 0.0, 0.6],
                    [1.2, 1.0, 0.5, 0.0, 0.0, 0.7],
                    [1.0, 1.0, 0.0, 0.0, 0.0, -0.5],
                    [1.1, 1.0, 0.0, 0.0, 0.0, -0.6],
                    [1.2, 1.0, 0.0, 0.0, 0.0, -0.7]
                ]
        },
        'base_params': {
            'i': 0,
            'name': 'cheese_scatter',
            'rev': False,
            'frame': 'base_link',
        }
    }

    circle_d = {

        'pattern_params': {
            'pattern_type': 'circular',
            'r': 0.5,
            'num_points': 4,
        },
        'base_params': {
            'i': 0,
            'name': 'cheese_circle',
            'rev': False,
            'frame': 'base_link',
        }
    }

    interface = Interface([linear_d, linear_d2, rect_d, scatter_d, circle_d])

    g_id0 = interface.group(
        [
            interface.patterns[0],
            interface.patterns[1],
            interface.patterns[2]
        ]
    )

    g_id1 = interface.group( 
        [
            interface.patterns[3],
            interface.patterns[4]
        ]
    )

    g0 = interface.groups[g_id0]
    g1 = interface.groups[g_id1]

    print "group id: {} | length: {} | element ids: {} | elements type: {} \
        \ngroup id: {} | length: {} | element ids: {} | elements type: {}".format(
        g_id0,
        g0.element_count,
        g0.elements.keys(),
        g0.elements[0].__class__.__base__.__name__,
        g_id1,
        g1.element_count,
        g1.elements.keys(),
        g1.elements[0].__class__.__base__.__name__,
    )

    g_id2 = interface.group(
        [
            g0,
            g1
        ]
    )

    g2 = interface.groups[g_id2]

    print "group id: {} | length: {} | element ids: {} | elements type: {}".format(
        g_id2,
        g2.element_count,
        g2.elements.keys(),
        type(g2.elements[0]).__name__
    )

    print "all patterns: {}".format(interface.patterns.keys())
    print "all groups: {}".format(interface.groups.keys())

    while g2.has_finished() is not True:
        print "g2 - current element: {} | next element: {}".format(g2.get_current_element(), g2.get_next_element())
        g2.increase_iterator()

    return interface


if __name__ == '__main__':
    intf = intf_example()