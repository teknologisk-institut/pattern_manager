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

from pattern_manager.collection import ContainerGroup, ContainerPattern, Container, Manager
from pattern_manager.patterns import PatternFactory, Pattern
from pluginlib import PluginLoader


if __name__ == '__main__':
    # Load pattern plugins
    _ld = PluginLoader(group='patterns')
    for k in _ld.plugins['pattern'].keys():
        PatternFactory.reg_pattern_typ(k, _ld.get_plugin('pattern', k))

    # Make root group to contain all subsequent groups
    c_root = ContainerGroup('root')
    Manager.set_active(c_root, True)

    # Create and add subgroups to root group
    c1 = ContainerPattern('c1', c_root)
    c2 = ContainerGroup('c2', c_root)

    import pattern_manager.examples as ex

    # Create new pattern from dict
    p1 = PatternFactory.mk_pattern(
        ex.linear_d['pattern_type'],
        ex.linear_d['base_params'],
        ex.linear_d['pattern_params'])

    # Add pattern to group g1
    c1.add_child(p1)

    Manager.set_active(p1, True)

    actv_elements = Manager.get_active_subs(c_root, incl_self=True)
    actv_element_names = []
    for e in actv_elements:
        actv_element_names.append(e.name)

    print "Active elements: {}".format(actv_element_names)

    actv_leaf = Manager.get_active_leaf(c_root)
    print "\nActive leaf: {}".format(actv_leaf.name)

    print "\nTree of container c_root:"
    Container.print_tree(c_root)

    print "\nTree of container c1:"
    Container.print_tree(c1)

    print '\nContainer by name: c1 | {}'.format(Container.get_container_by_name('c1'))

    print '\nPattern by name: linear | {}'.format(Pattern.get_pattern_by_name('cheese_linear'))

    print '\nIterating through active patterns: '
    while actv_leaf and isinstance(actv_leaf, Pattern):
        print actv_leaf.name, Manager.i[id(actv_leaf)]

        Manager.iterate(actv_leaf)

        if Manager.finished[id(actv_leaf)]:
            actv_leaf = Manager.get_active_leaf(c_root)

    print '\nDone...'

    print ''
