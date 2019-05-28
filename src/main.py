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

from pattern_manager.collection import Group, Manager

if __name__ == '__main__':
    mgr = Manager.getInstance()
    g_root = Group("root")
    mgr.set_active(g_root.id, True)

    g1 = Group("g1")
    g2 = Group("g2")

    g_root.add_subelement(g1)
    g_root.add_subelement(g2)

    mgr.set_active_subs(g_root, True)

    # Printing groups under root:
    for g in g_root.elements:
        print "    {}".format(g.name)

    # print "\nfetching active group:"
    # leaf = mgr.get_active_leaf(g_root)
    # print "    active group: {}".format(leaf.name)
    # mgr.set_active(g1.id, False)
    # print "    (setting {} to inactive)".format(g1.name)
    # leaf = mgr.get_active_leaf(g_root)
    # print "    active group: {}\n".format(leaf.name)

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

    g1.add_subelement(p1)
    mgr.set_active(p1.id, True)

    cheese1 = g_root.find_subelement_by_nm("cheese_linear", g_root)

    print cheese1.name

    print mgr.get_active_leaf(g_root).name
