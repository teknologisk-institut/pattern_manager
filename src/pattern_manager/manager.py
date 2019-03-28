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
    _patterns = {}  # dict of patterns
    _fitters = {}  # ICP pattern fitters corresponding to pattern ID
    _patterns_are_grouped = True  # whether or not all patterns are grouped (i.e. if the iterator overflows to next pattern or not)
    _active_pattern = 0  # index of last pattern used
    _factory = PatternFactory()
    _layers = {}

    def __init__(self, grouped_patterns=True):
        self.set_grouped_patterns(grouped_patterns)
        self.loader = PluginLoader(group='patterns')
        self._load_plugins()

    def _load_plugins(self):
        for k in self.loader.plugins['pattern'].keys():
            self._factory.register_pattern_type(k, self.loader.get_plugin('pattern', k))

    def add_pattern(self, pattern):
        if len(self._patterns) == 0:
            i = 0
        else:
            i = max(self._patterns.keys()) + 1
        self._patterns[i] = pattern
        return i

    def remove_pattern(self, pattern_index):
        del self._patterns[pattern_index]

    def create_pattern_from_dict(self, pattern_type, pattern_params, base_params, group_id, layer):
        pattern = self._factory.get_pattern(pattern_type, base_params, pattern_params)

        if layer in self._layers:
            if group_id in self._layers[layer]:
                self._layers[layer][group_id].append(pattern)
            else:
                self._layers[layer][group_id] = [pattern]
        else:
            self._layers[layer] = {group_id : [pattern]}

        return pattern

    # def create_child_pattern(self, pattern_dict, child_dict, name_suffix=""):
    #     child_args = child_dict
    #     child_args['frame_id'] = pattern_dict['frame_id']
    #     child_args['pattern_name'] = pattern_dict['pattern_name'] + name_suffix
    #     p = self.create_pattern_from_dict(child_args)
    #     return p

    def get_pattern_count(self):
        return len(self._patterns)

    def set_grouped_patterns(self, grouped):
        self._patterns_are_grouped = grouped

    def get_pattern_indices(self):
        return self._patterns.keys()

    # Inidvidual pattern access

    def is_pattern_finished(self, pattern_index):
        return self._patterns[pattern_index].is_finished()

    def reset_pattern(self, pattern_index):
        return self._patterns[pattern_index].reset_pattern()

    def get_iterator(self, pattern_index):
        return self._patterns[pattern_index].get_iterator()

    def set_iterator(self, pattern_index, iterator):
        self._patterns[pattern_index].set_iterator(iterator)

    def is_reverse_iteration(self, pattern_index):
        return self._patterns[pattern_index].is_reverse_iteration()

    def set_reverse_iteration(self, pattern_index, reverse):
        return self._patterns[pattern_index].set_reverse_iteration(reverse)

    def get_pattern_size(self, pattern_index):
        return self._patterns[pattern_index].get_pattern_size()

    def get_pattern_frame_id(self, pattern_index):
        return self._patterns[pattern_index].get_pattern_frame_id()

    def get_pattern_name(self, pattern_index):
        return self._patterns[pattern_index].get_pattern_name()

    def set_pattern_name(self, pattern_index, new_name):
        return self._patterns[pattern_index].set_pattern_name(new_name)

    def get_pattern(self, pattern_index):
        return self._patterns[pattern_index]._pattern

    def get_current_tf_in_pattern(self, pattern_index):
        return self._patterns[pattern_index].get_current_tf()

    # smart pattern interaction using extra classes
    def update_pattern(self, pattern_index, poses, fit_input_to_pattern, overwrite_orientation):
        print "%s input poses" % len(poses)
        if fit_input_to_pattern:
            # only create the pattern fitter when we need it
            if pattern_index not in self._fitters:
                self._fitters[pattern_index] = pattern_fitter.PatternFitter(self._patterns[pattern_index])
        self._fitters[pattern_index].update_pattern(poses, fit_input_to_pattern, overwrite_orientation)
        self._patterns[pattern_index] = self._fitters[pattern_index].get_updated_pattern()
        return True

    # internal manager functions

    def find_pattern_by_name(self, name, exact_match=True):
        indices = self.get_pattern_indices()
        matches = []
        for i in indices:
            p_name = self.get_pattern_name(i)
            if exact_match:
                if p_name == name:
                    matches.append(i)
            else:
                if name.lower() in p_name.lower():
                    matches.append(i)
        if len(matches) == 0:  # no matches
            return False
        elif len(matches) == 1:  # one match
            return matches[0]
        else:  # multiple matches
            finished = [self.is_pattern_finished(i) for i in matches]
            unstarted = [self.get_iterator(i) == 0 for i in matches]
            if all(finished) or all(unstarted):
                return matches[0]
            else:
                processing = [((not finished[i]) and (not unstarted[i])) for i in range(len(matches))]
                if True in processing:
                    # TODO: don't just return the first match?
                    return matches[processing.index(True)]
                else:
                    return matches[unstarted.index(True)]

    def set_active_pattern(self, pattern_index):
        if pattern_index in self._patterns.keys():
            self._active_pattern = pattern_index
            return True
        else:
            return False

    def sorted_pattern_ids(self):
        return sorted(self._patterns.keys())

    def get_current_pattern(self):
        # not using groups?
        if not self._patterns_are_grouped:
            return self._active_pattern, self._patterns[self._active_pattern]
        # we're currently working on a pattern
        if not self.is_pattern_finished(self._active_pattern):
            return self._active_pattern, self._patterns[self._active_pattern]
        # get first available unfinished pattern
        (k, p) = self.get_first_unfinished_pattern()
        if k is False:
            return k, p
        # if all patterns are finished
        return self._active_pattern, self._patterns[self._active_pattern]

    def get_first_unfinished_pattern(self):
        # get first available unfinished pattern
        keys = self.sorted_pattern_ids()
        for k in keys:
            if self.is_pattern_finished(k):
                continue
            else:
                self._active_pattern = k
                return k, self._patterns[k]
        return (False, False)

    # manager interaction

    def get_current_tf(self):
        (i, p) = self.get_current_pattern()
        return p.get_current_tf()

    def get_next_tf(self):
        (i, p) = self.get_current_pattern()
        next_pattern_i = p.get_iterator() + 1
        if not next_pattern_i < p.get_pattern_size():
            # using groups?
            if not self._patterns_are_grouped:
                return self.get_current_tf()
            # advance to next pattern
            next_index = self._patterns.keys().index(i) + 1
            p = self._patterns[next_index]
            return p.get_tf_from_iter(0)
        else:
            return p.get_next_tf()

    def get_tf_from_iter(self, pattern, iter):
        return self._patterns[pattern].get_tf_from_iter(iter)

    def increase_iterator(self):
        (i, p) = self.get_current_pattern()
        pattern_i = p.increase_iterator()
        # print pattern_i
        if pattern_i is False and self._patterns_are_grouped:
            (i, p) = self.get_current_pattern()
            if i is False:
                return i
            else:
                pattern_i = 0
        return pattern_i


if __name__ == '__main__':
    man = PatternManager()
    linear_d = {
        'pattern_type': 'linear',
        'layer': 0,
        'group_id': 0,
        'pattern_params': {
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
        'pattern_type': 'rectangular',
        'layer': 0,
        'group_id': 0,
        'pattern_params': {
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
        'pattern_type': 'scatter',
        'layer': 1,
        'group_id': 1,
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
            'i': 0,
            'name': 'cheese_scatter',
            'rev': False,
            'frame': 'base_link',
        }
    }

    circle_d = {
        'pattern_type': 'circular',
        'layer': 1,
        'group_id': 2,
        'pattern_params': {
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

    pat_linear = man.create_pattern_from_dict(
        linear_d['pattern_type'],
        linear_d['pattern_params'],
        linear_d['base_params'],
        linear_d['group_id'],
        linear_d['layer']
    )

    pat_rect = man.create_pattern_from_dict(
        rect_d['pattern_type'],
        rect_d['pattern_params'],
        rect_d['base_params'],
        rect_d['group_id'],
        rect_d['layer']
    )

    pat_scatter = man.create_pattern_from_dict(
        scatter_d['pattern_type'],
        scatter_d['pattern_params'],
        scatter_d['base_params'],
        scatter_d['group_id'],
        scatter_d['layer']
    )

    pat_circle = man.create_pattern_from_dict(
        circle_d['pattern_type'],
        circle_d['pattern_params'],
        circle_d['base_params'],
        circle_d['group_id'],
        circle_d['layer']
    )

    pat_list = [pat_linear, pat_rect, pat_scatter, pat_circle]

    for layer in man._layers.keys():
        print 'layer:', layer
        for group in man._layers[layer].keys():
            print '  group:', group
            for pattern in man._layers[layer][group]:
                print '    ', pattern.pattern_name