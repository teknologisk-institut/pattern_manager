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

class Manager(object):

    _instance = None
    id = 0

    @staticmethod
    def getInstance():
        if Manager._instance is None:
            Manager()
        return Manager._instance

    def __init__(self):
        if Manager._instance is not None:
            raise Exception(
                "Instance already exists - this class is a singleton")
        else:
            Manager._instance = self

        self.i = [0] * 20
        self.finished = [False] * 20
        self.active = [False] * 20
        # TODO: iteration order for each group -- self.iter_ordr[]

    def iterate(self, e):
        self.i[e.id] += 1

        if not self.i[e.id] < e.element_cnt:
            self.set_finished(e.id, True)
            self.set_active(e.id, False)

            return False

        return True

    def reset_e_mgr(self, e_id):
        self.i[e_id] = 0
        self.finished[e_id] = False

    def set_active(self, e_id, actv):
        self.active[e_id] = actv

    def set_finished(self, g_id, fin):
        self.finished[g_id] = fin

    def get_active_leaf(self, e):
        if not self.active[e.id]:
            return None
            
        if not hasattr(e, 'elements') or e.element_cnt == 0:
            return e
            
        for e in e.elements:
            if self.active[e.id]:
                return self.get_active_leaf(e)

        return e

    def set_active_supers(self, e, actv):
        while e.par:
            self.active[e.id] = actv

        return

    def set_active_subs(self, e, actv):
        for g in e.elements:
            self.active[g.id] = actv
            self.set_active_subs(g, actv)
