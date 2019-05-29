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
    def __init__(self, typ, nm):
        self.typ = typ
        self.nm = nm
        self.id = Manager.id
        self.grps = []
        self.pats = []
        self.par = None
        Manager.id += 1

    def add_subgroup(self, grp):
        self.grps.append(grp)
        grp.par = self

    def add_pattern(self, pat):
        if not self.typ is GType.pattern:
            print "Error: cannot add pattern to group of type, {}".format(
                self.typ.name)

        self.pats.append(pat)
        pat.par = self

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
