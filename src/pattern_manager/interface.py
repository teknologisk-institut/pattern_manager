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

        self._factory = PatternFactory()
        self.loader = PluginLoader(group='patterns')
        self._load_pattern_types()
        self.patterns = {}
        self.groups = {}
        self.id = 0

        for i in range(len(pattern_dicts)):
            self.patterns[i] = self.create_pattern_from_dict(
                pattern_dicts[i]['pattern_params'],
                pattern_dicts[i]['base_params']
            )


    def _load_pattern_types(self):
        for k in self.loader.plugins['pattern'].keys():
            self._factory.register_pattern_type(
                k, self.loader.get_plugin('pattern', k))

    def create_pattern_from_dict(self, pattern_params, base_params):
        pattern_type = pattern_params.pop('pattern_type')

        return self._factory.get_pattern(pattern_type, base_params, pattern_params)

    def group(self, elements):
        id = copy(self.id)
        manager = Manager(elements)
        self.groups[id] = manager
        self.id += 1

        return id

    def ungroup(self, id):
        try:
            del self.groups[id]
        except KeyError:
            return False


if __name__ == '__main__':
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

    print "group 1 id: {} | length: {} | ids: {} | elements type: {}\ngroup 2 id: {} | length: {} | ids: {} | elements type: {}".format(
        g_id0,
        g0.element_count(),
        g0.elements.keys(),
        g0.elements[0].__class__.__base__.__name__,
        g_id1,
        g1.element_count(),
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

    print "group 3 id: {} | length: {} | ids: {} | elements type: {}".format(
        g_id2,
        g2.element_count(),
        g2.elements.keys(),
        type(g2.elements[0]).__name__
    )

    print "all patterns: {}".format(interface.patterns.keys())
    print "all groups: {}".format(interface.groups.keys())

    print "> remove group 2"
    interface.ungroup(g_id2)

    print "all groups: {}".format(interface.groups.keys())
