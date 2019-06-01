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

from abc import ABCMeta


class Manager(object):
    __metaclass__ = ABCMeta

    i = {}
    finished = {}
    active = {}
    # TODO: iteration order for each group -- self.iter_ordr[]

    @staticmethod
    def register_id(id):
        Manager.i[id] = 0
        Manager.finished[id] = False
        Manager.active[id] = False

    @staticmethod
    def iterate(e):
        nxt_i = Manager.i[id(e)] + 1

        count = 0
        if e.typ == "Pattern":
            count = len(e.tfs)
        elif e.typ == "Group":
            count = len(e.chldrn)

        if not nxt_i < count:
            Manager.set_finished(id(e), True)
            Manager.set_active(id(e), False)

            if e.par:
                Manager.iterate(e.par)

            return False

        Manager.i[id(e)] = nxt_i

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
        for g in grp.chldrn:
            if g.typ == "Group" and Manager.active[id(g)]:
                return Manager.get_active_group(g)

        return grp

    @staticmethod
    def get_active_pattern(actv_grp):
        if not actv_grp.g_typ == "GOP" or actv_grp.child_cnt == 0:
            return None

        i = Manager.i[id(actv_grp)]

        return actv_grp.chldrn[i]

    @staticmethod
    def set_active_supers(e, actv):
        while e.par:
            Manager.active[id(e.par)] = actv
            e = e.par

        return True

    @staticmethod
    def set_active_subs(grp, actv):
        for g in grp.chldrn:
            Manager.active[id(g)] = actv

            if grp.g_typ == "GOG":
                Manager.set_active_subs(g, actv)

    @staticmethod
    def set_active_pattern(pat):
        Manager.set_active(id(pat), True)
        Manager.set_active_supers(pat, True)
