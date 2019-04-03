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

# from patterns import PatternLinear, PatternRectangular, PatternScatter
# import pattern_manager.pattern_fitter as pattern_fitter
from pluginlib import PluginLoader
from pattern_manager.patterns import pattern_base
from pattern_manager.containers import PatternGroup

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


class PatternManager(object):
    _factory = PatternFactory()
    _layers = {}

    def __init__(self, grouped_patterns=True):
        self.loader = PluginLoader(group='patterns')
        self._load_patterns()
        self.i = 0

    def _load_patterns(self):
        for k in self.loader.plugins['pattern'].keys():
            self._factory.register_pattern_type(k, self.loader.get_plugin('pattern', k))

    def add_pattern(self, pattern, layer, group_id):
        if layer in self._layers:
            if group_id in self._layers[layer].keys():
                self._layers[layer][group_id].add_pattern(pattern)
            else:
                self._layers[layer][group_id] = PatternGroup(pattern)
        else:
            self._layers[layer] = {group_id: PatternGroup(pattern)}

    def remove_pattern(self, pattern, layer, group_id):
        try:
            self._layers[layer][group_id].remove_pattern(pattern)
            return True
        except KeyError:
            print 'error: pattern not found'
            return False

    def create_pattern_from_dict(self, pattern_type, pattern_params, base_params):
        layer = 0 if 'layer' not in base_params.keys() else base_params.pop('layer')
        group_id = 0 if 'group_id' not in base_params.keys() else base_params.pop('group_id')

        pattern = self._factory.get_pattern(pattern_type, base_params, pattern_params)

        self.add_pattern(pattern, layer, group_id)


if __name__ == '__main__':
    man = PatternManager()
    linear_d = {
        'pattern_type': 'linear',
        'pattern_params': {
            'step_size': 0.1,
            'num_points': 3
        },
        'base_params': {
            'layer': 0,
            'group_id': 0,
            'i': 0,
            'name': 'cheese_linear',
            'rev': False,
            'frame': 'base_link',
            'offset_xy': [0.5, 0.3],
            'offset_rot': 0.2,
        }
    }

    linear_d2 = {
        'pattern_type': 'linear',
        'pattern_params': {
            'step_size': 0.1,
            'num_points': 3
        },
        'base_params': {
            'layer': 0,
            'group_id': 0,
            'i': 0,
            'name': 'cheese_linear',
            'rev': False,
            'frame': 'base_link',
            'offset_xy': [0.5, 0.3],
            'offset_rot': 0.2,
        }
    }

    rect_d = {
        'pattern_type': 'rectangular',
        'pattern_params': {
            'num_points': (3, 2),
            'step_sizes': (0.1, 0.1),
        },
        'base_params': {
            'layer': 0,
            'group_id': 1,
            'i': 0,
            'name': 'cheese_rect',
            'rev': False,
            'frame': 'base_link',
        }
    }

    scatter_d = {
        'pattern_type': 'scatter',
        'pattern_params': {
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
            'layer': 1,
            'group_id': 1,
            'i': 0,
            'name': 'cheese_scatter',
            'rev': False,
            'frame': 'base_link',
        }
    }

    circle_d = {
        'pattern_type': 'circular',
        'pattern_params': {
            'r': 0.5,
            'num_points': 4,
        },
        'base_params': {
            'layer': 1,
            'group_id': 1,
            'i': 0,
            'name': 'cheese_circle',
            'rev': False,
            'frame': 'base_link',
        }
    }

    pat_linear = man.create_pattern_from_dict(
        linear_d['pattern_type'],
        linear_d['pattern_params'],
        linear_d['base_params']
    )

    pat_linear2 = man.create_pattern_from_dict(
        linear_d2['pattern_type'],
        linear_d2['pattern_params'],
        linear_d2['base_params']
    )

    pat_rect = man.create_pattern_from_dict(
        rect_d['pattern_type'],
        rect_d['pattern_params'],
        rect_d['base_params']
    )

    pat_scatter = man.create_pattern_from_dict(
        scatter_d['pattern_type'],
        scatter_d['pattern_params'],
        scatter_d['base_params']
    )

    pat_circle = man.create_pattern_from_dict(
        circle_d['pattern_type'],
        circle_d['pattern_params'],
        circle_d['base_params']
    )

    pat_list = [pat_linear, pat_linear2, pat_rect, pat_scatter, pat_circle]

    
