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

from pattern_manager.collection import GType, Group, Manager
from pattern_manager.patterns import PatternFactory
from pluginlib import PluginLoader


if __name__ == '__main__':
    # Load pattern plugins
    _ld = PluginLoader(group='patterns')
    for k in _ld.plugins['pattern'].keys():
        PatternFactory.reg_pattern_typ(k, _ld.get_plugin('pattern', k))

    # Make root group to contain all subsequent groups
    g_root = Group(GType.GOG, "root")
    Manager.set_active(id(g_root), True)

    # Create and add subgroups to root group
    g1 = Group(GType.GOP, "g1", g_root)
    g2 = Group(GType.GOG, "g2", g_root)

    import pattern_manager.examples as ex

    # Create new pattern from dict
    p1 = PatternFactory.mk_pattern(
        ex.linear_d['pattern_type'],
        ex.linear_d['base_params'],
        ex.linear_d['pattern_params'])

    # Add pattern to group g1
    g1.add_child(p1)

    print "All elements:"
    Group.print_tree(g_root)

    Manager.set_active_pattern(p1)

    print "\nActive elements:"
    Manager.print_active_subs(g_root)

    print "\nIterating through active pattern transforms..."
    actv_pat = Manager.get_active_pattern(g_root)
    while actv_pat:
        print actv_pat.nm, Manager._i[id(actv_pat)]

        Manager.iterate(actv_pat)

        if Manager._finished[id(actv_pat)]:
            actv_pat = Manager.get_active_pattern(g_root)

    print Group.get_sub_by_name("cheese_linear", g_root)