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
from container import Container


class Manager(object):
    __metaclass__ = ABCMeta

    i = {}
    finished = {}
    active = {}

    @staticmethod
    def register_id(id):
        Manager.i[id] = 0
        Manager.finished[id] = False
        Manager.active[id] = False

    @staticmethod
    def iterate(e):
        nxt_i = Manager.i[id(e)] + 1
        count = e.child_count()

        if nxt_i < count:
            Manager.i[id(e)] = nxt_i

            return True

        Manager.set_finished(id(e), True)
        Manager.set_active(e, False)

        if e.parent:
            Manager.iterate(e.parent)

        return False

    @staticmethod
    def reset_element(id):
        Manager.i[id] = 0
        Manager.finished[id] = False

    @staticmethod
    def set_finished(id, fin):
        Manager.finished[id] = fin

    @staticmethod
    def get_active_leaf(root):

        if isinstance(root, Container):
            for c in root.children:

                if not Manager.active[id(c)]:
                    continue

                return Manager.get_active_leaf(c)

        return root

    @staticmethod
    def set_active(e, actv):
        Manager.active[id(e)] = actv
        Manager.set_active_supers(e, actv)

    @staticmethod
    def set_active_supers(e, actv):

        while e.parent:
            Manager.active[id(e.parent)] = actv
            e = e.parent

        return True

    @staticmethod
    def get_active_subs(e, incl_self=False):
        actv_subs = []

        if incl_self:
            actv_subs.append(e)

        while isinstance(e, Container):
            for c in e.children:

                if not Manager.active[id(c)]:
                    continue

                actv_subs.append(c)
                e = c

                break

        return actv_subs

    @staticmethod
    def set_active_subs(e, actv):

        for c in e.children:
            Manager.active[id(c)] = actv

            if not isinstance(c, Container):
                continue

            Manager.set_active_subs(c, actv)

    @staticmethod
    def reset_subs(e):

        for c in e.children:
            Manager.reset_element(id(c))

            if not isinstance(c, Container):
                continue

            Manager.reset_subs(c)
