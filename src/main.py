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

# from pattern_manager.manager import Manager
# from pattern_manager.group import Group
from pattern_manager.patterns import PatternFactory
from pluginlib import PluginLoader
from pattern_manager.tree import Tree


if __name__ == '__main__':
    t = Tree('root', None)
    t1 = Tree('t1', t)
    t2 = Tree('t2', t)
    t3 = Tree('t3', t)

    t4 = Tree('t4', t2)
    t5 = Tree('t5', t2)

    t.active = True
    t2.active = True
    t5.active = True

    ld = PluginLoader(group='patterns')
    for k in ld.plugins['pattern'].keys():
        PatternFactory.reg_pattern_typ(k, ld.get_plugin('pattern', k))

    import pattern_manager.examples as ex

    p1 = PatternFactory.mk_pattern(
        ex.linear_d['pattern_type'],
        ex.linear_d['base_params'],
        ex.linear_d['pattern_params'])

    t4.add_pattern(p1)

    for n in Tree.get_active_nodes():
        print n.name

    print 'Found tree with id %d: %s' % (id(t5), Tree.get_node(id(t5)).name)

    print Tree.get_pattern(id(p1))
