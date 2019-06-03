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
import Queue

class GType(Enum):
    GOG = 1
    GOP = 2


class Group(object):
    def __init__(self, g_typ, nm, par=None):
        self.g_typ = g_typ.name
        self.typ = self.__class__.__name__
        self.nm = nm
        self.chldrn = []
        self.par = par

        if self.par:
            par.add_child(self)

        Manager.register_id(id(self))

    def add_child(self, chld):
        if (self.g_typ == "GOP" and chld.typ == "Group") or \
                (self.g_typ == "GOG" and chld.typ == "Pattern"):
            return False

        self.chldrn.append(chld)
        chld.par = self

        return True
    
    @staticmethod
    def get_sub_by_name(nm, root):
        q = Queue.Queue()
        q.put(root)

        while not q.empty():
            node = q.queue[0]
            if node.nm == nm:
                return node

            q.get()

            for sub in node.chldrn:
                q.put(sub)

        return None

    @staticmethod
    def print_tree(chld):
        print id(chld), chld.nm

        if chld.typ == "Group":
            for sub in chld.chldrn:
                Group.print_tree(sub)

    def child_cnt(self):
        len(self.chldrn)
