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


class ManagerNew(object):

    _instance = None

    @staticmethod
    def getInstance():
        if ManagerNew._instance is None:
            ManagerNew()
        return ManagerNew._instance

    def __init__(self):
        if ManagerNew._instance is not None:
            raise Exception("Instance already exists - this class is a singleton")
        else:
            ManagerNew._instance = self

        self.i = [0] * 20
        self.nxt_i = [1] * 20
        self.finished = [False] * 20
        self.active = [False] * 20

    def iterate(self, g_id):
        self.i[g_id] += 1
        self.nxt_i[g_id] += 1

    def reset_mgr(self, g_id):
        self.i[g_id] = 0
        self.nxt_i[g_id] = 1
        self.finished[g_id] = False

    def set_active(self, g_id, actv):
        self.active[g_id] = actv

    def set_finished(self, g_id, fin):
        self.finished[g_id] = fin

    def get_active_leaf(self, grp):        
        for g in grp.grps:
            if self.active[g.id]:
                return self.get_active_leaf(g)
        
        return grp

    def set_active_supers(self, grp, actv):
        while grp.par:
            self.active[grp.id] = actv

        return

    def set_active_subs(self, grp, actv):
        for g in grp.grps:
            self.active[g.id] = actv
            self.set_active_subs(g, actv)
    
