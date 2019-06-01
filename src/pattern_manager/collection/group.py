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
    GOG = 1
    GOP = 2


class Group(object):
    def __init__(self, g_typ, nm):
        self.g_typ = g_typ.name
        self.typ = self.__class__.__name__
        self.nm = nm
        self.chldrn = []
        self.par = None

        Manager.register_id(id(self))

    def add_child(self, chld):
        self.chldrn.append(chld)
        chld.par = self

    def find_successor_by_nm(self, nm, rslt, chld):
        if chld:
            if chld.nm == nm:
                rslt = chld
                return

            for sub in chld.chldrn:
                self.find_successor_by_nm(sub, nm, rslt)

    def child_cnt(self):
        len(self.chldrn)
