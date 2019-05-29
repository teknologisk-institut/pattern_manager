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

if __name__ == '__main__':
    mgr = Manager.getInstance()
    g_root = Group(GType.group, "root")
    mgr.set_active(g_root.id, True)

    g1 = Group(GType.pattern, "g1")
    g2 = Group(GType.group, "g2")

    g_root.add_subgroup(g1)
    g_root.add_subgroup(g2)

    mgr.set_active_subs(g_root, True)

    # for g in g_root.grps:
    #     print g.nm
    #     print mgr.active[g.id]

    leaf = mgr.get_active_leaf(g_root)
    print leaf.nm
    mgr.set_active(g1.id, False)
    leaf = mgr.get_active_leaf(g_root)
    print leaf.nm

    from pattern_manager.patterns import PatternFactory
    from pluginlib import PluginLoader
    import pattern_manager.examples as ex

    _ld = PluginLoader(group='patterns')
    for k in _ld.plugins['pattern'].keys():
        PatternFactory.reg_pattern_typ(k, _ld.get_plugin('pattern', k))

    p1 = PatternFactory.mk_pattern(
        ex.linear_d['pattern_type'],
        ex.linear_d['base_params'],
        ex.linear_d['pattern_params'])

    print p1.nm
