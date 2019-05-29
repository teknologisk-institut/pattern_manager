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

# from pattern_manager.collection import GType
# from pattern_manager.patterns import Pattern
# from pattern_manager.collection import Group
from pattern_manager.collection import Group
from abc import ABCMeta


class Manager(object):
    __metaclass__ = ABCMeta

    id = 0
    i = [0] * 20
    finished = [False] * 20
    active = [False] * 20
    # TODO: iteration order for each group -- self.iter_ordr[]

    @staticmethod
    def iterate(e):
        Manager.i[e.id] += 1

        count = 0
        if hasattr(e, "tfs"):
            count = len(e.tfs)
        elif hasattr(e, "grps"):
            if e.typ == 1:
                count = len(e.grps)
            elif e.typ == 2:
                count = len(e.pats)

        if not Manager.i[e.id] < count:
            Manager.set_finished(e.id, True)
            Manager.set_active(e.id, False)

            if e.par:
                Manager.iterate(e.par)

            return False

        return True

    @staticmethod
    def reset_mgr(id):
        Manager.i[id] = 0
        Manager.finished[id] = False

    @staticmethod
    def set_active(id, actv):
        Manager.active[id] = actv

    @staticmethod
    def set_finished(id, fin):
        Manager.finished[id] = fin

    @staticmethod
    def get_active_group(grp):
        for g in grp.grps:
            if Manager.active[g.id]:
                return Manager.get_active_group(g)

        return grp

    @staticmethod
    def get_active_pattern(actv_grp):
        if actv_grp.pattern_cnt == 0:
            return False

        return actv_grp.pats[Manager.i[actv_grp.id]]

    @staticmethod
    def set_active_supers(e, actv):
        while e.par:
            Manager.active[e.par.id] = actv
            e = e.par

        return

    @staticmethod
    def set_active_subs(grp, actv):
        if grp.typ == 1:
            for g in grp.grps:
                Manager.active[g.id] = actv
                Manager.set_active_subs(g, actv)
        elif grp.typ == 2:
            for p in grp.pats:
                Manager.active[p.id] = actv

    @staticmethod
    def set_active_pattern(pat):
        Manager.set_active(pat.id, True)
        Manager.set_active_supers(pat, True)
