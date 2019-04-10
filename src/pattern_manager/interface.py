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
    _factory = PatternFactory()

    def __init__(self, grouped_patterns=True):
        self.loader = PluginLoader(group='patterns')
        self._load_pattern_types()
        self.layers = {}

    def _load_pattern_types(self):
        for k in self.loader.plugins['pattern'].keys():
            self._factory.register_pattern_type(k, self.loader.get_plugin('pattern', k))

    def create_pattern_from_dict(self, pattern_params, base_params):
        pattern_type = pattern_params.pop('pattern_type')
        layer = 0 if 'layer' not in base_params.keys() else base_params.pop('layer')
        group_id = 0 if 'group_id' not in base_params.keys() else base_params.pop('group_id')

        pattern = self._factory.get_pattern(pattern_type, base_params, pattern_params)

        g = Manager()
        if layer in self.layers.keys():
            if group_id in self.layers[layer].elements.keys():
                g = self.layers[layer].elements[group_id]
                g.add_element(pattern, g.element_count(), pattern.pattern_name)
            else:
                g.add_element(pattern, 0, pattern.pattern_name)
                self.layers[layer].elements[group_id] = g
        else:
            g.add_element(pattern, 0, pattern.pattern_name)
            l = Manager()
            l.add_element(g, group_id)
            self.layers[layer] = l
            

if __name__ == '__main__':
    man = Interface()

    linear_d = {
        'pattern_params': {
            'pattern_type': 'linear',
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
        'pattern_params': {
            'pattern_type': 'linear',
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
        'pattern_params': {
            'pattern_type': 'rectangular',
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
            'layer': 1,
            'group_id': 1,
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
            'layer': 1,
            'group_id': 1,
            'i': 0,
            'name': 'cheese_circle',
            'rev': False,
            'frame': 'base_link',
        }
    }

    pat_dict_list = [linear_d, linear_d2, rect_d, scatter_d, circle_d]

    for d in pat_dict_list:
        man.create_pattern_from_dict(
            d['pattern_params'],
            d['base_params']
        )

    for l in man.layers.keys():
        print 'layer: {}'.format(l)
        for g in man.layers[l].elements.keys():
            print '  group: {}'.format(g)
            for p in man.layers[l].elements[g].elements.keys():
                print '    pattern {}: {}:'.format(p, man.layers[l].elements[g].names[p])
                for c in man.layers[l].elements[g].elements[p]._pattern:
                    print '      point: {}, {}, {}\n      rotation: {}, {}, {}, {}'.format(
                        c.translation.x, 
                        c.translation.y,
                        c.translation.z,
                        c.rotation.w, 
                        c.rotation.x,
                        c.rotation.y,
                        c.rotation.z)
