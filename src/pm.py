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

# class PatternManager():
#     def __init__(self):
#         self.g_root = Group(GType.group, "root")
#         Manager.set_active(g_root, True)

#     def make_group(self, typ, nm):
#         grp = Group(typ, nm)
#         self.g_root.add_subgroup(grp)

#         return grp

#     def make_pattern(self, dict):
#         pat = PatternFactory.mk_pattern(
#             dict['pattern_type'], 
#             dict['base_params'], 
#             dict['pattern_params'])

#         return pat

if __name__ == '__main__':
    from pattern_manager.patterns import PatternFactory
    from pluginlib import PluginLoader

    # Load pattern plugins
    _ld = PluginLoader(group='patterns')
    for k in _ld.plugins['pattern'].keys():
        PatternFactory.reg_pattern_typ(k, _ld.get_plugin('pattern', k))

    # Make root group to contain all subsequent groups
    g_root = Group(GType.GOG, "root")
    Manager.set_active(id(g_root), True)

    g1 = Group(GType.GOP, "g1")
    g2 = Group(GType.GOG, "g2")

    # Add new groups to root group
    g_root.add_child(g1)
    g_root.add_child(g2)

    import pattern_manager.examples as ex

    # Create new pattern from dict
    p1 = PatternFactory.mk_pattern(
        ex.linear_d['pattern_type'],
        ex.linear_d['base_params'],
        ex.linear_d['pattern_params'])

    g1.add_child(p1)

    Manager.set_active_pattern(p1)

    actv_grp = Manager.get_active_group(g_root)
    print actv_grp
    actv_pat = Manager.get_active_pattern(actv_grp)
    print actv_pat

    Manager.iterate(actv_pat)
    Manager.iterate(actv_pat)
    Manager.iterate(actv_pat)

    actv_grp = Manager.get_active_group(g_root)
    print actv_grp
    actv_pat = Manager.get_active_pattern(actv_grp)
    print actv_pat
