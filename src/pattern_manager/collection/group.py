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

from pattern_manager.collection import Manager
from enum import Enum


class GType(Enum):
    group = 1
    pattern = 2


class Group(object):
    id = 0

    def __init__(self, typ, nm):
        self.typ = typ
        self.nm = nm
        self.id = Group.id
        self.grps = []
        self.pats = []
        self.par = None
        Group.id += 1

    def add_subgroup(self, grp):
        if not self.typ is GType.group:
            print "Error: cannot add subgroup to group of type, {}".format(
                self.typ.name)
            return False

        self.grps.append(grp)
        grp.par = self

    def add_pattern(self, pat):
        if not self.typ is GType.pattern:
            print "Error: cannot add pattern to group of type, {}".format(
                self.typ.name)

        self.pats.append(pat)

    def find_subgroup_by_nm(self, nm, rslt, grp=None):
        if not grp:
            grp = self

        if grp:
            if grp.nm == nm:
                rslt = grp
                return

            for sub in grp.grps:
                self.find_subgroup_by_nm(sub, nm, rslt)

    def pattern_cnt(self):
        len(self.pats)

    def group_cnt(self):
        len(self.grps)


if __name__ == "__main__":
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
