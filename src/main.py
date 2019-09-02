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

from pattern_manager.manager import Manager
from pattern_manager.group import Group
from pattern_manager.patterns import PatternFactory, Pattern
from pluginlib import PluginLoader


if __name__ == '__main__':
    # Load pattern plugins
    ld = PluginLoader(group='patterns')
    for k in ld.plugins['pattern'].keys():
        PatternFactory.reg_pattern_typ(k, ld.get_plugin('pattern', k))

    c_root = Group('root')
    c1 = Group('c1', c_root)
    c2 = Group('c2', c_root)

    import pattern_manager.examples as ex

    # Create new pattern from dict
    p1 = PatternFactory.mk_pattern(
        ex.linear_d['pattern_type'],
        ex.linear_d['base_params'],
        ex.linear_d['pattern_params'])

    # Add pattern to group g1
    c1.add_child(p1)

    print '\n{} group type: {}'.format(c_root.name, c_root.group_type)
    print '{} group type: {}'.format(c1.name, c1.group_type)
    print '{} group type: {}'.format(c2.name, c2.group_type)

    Manager.set_active(p1, True)

    actv_elements = Manager.get_active_subs(c_root, incl_self=True)
    actv_element_names = []
    for e in actv_elements:
        actv_element_names.append(e.name)

    print "\nActive elements: {}".format(actv_element_names)

    actv_leaf = Manager.get_active_leaf(c_root)
    print "\nActive leaf: {}".format(actv_leaf.name)

    print "\nTree of group c_root:"
    Group.print_tree(c_root)

    print "\nTree of group c1:"
    Group.print_tree(c1)

    print '\nGroup by name: c1 | {}'.format(Group.get_group_by_name('c1'))

    print '\nPattern by name: linear | {}'.format(Pattern.get_pattern_by_name('cheese_linear'))

    print '\nIterating through active patterns: '
    while actv_leaf and actv_leaf.type == 'Pattern':
        print actv_leaf.name, Manager.i[id(actv_leaf)]

        Manager.iterate(actv_leaf)

        if Manager.finished[id(actv_leaf)]:
            actv_leaf = Manager.get_active_leaf(c_root)

    print '\nDone...'

    print ''
