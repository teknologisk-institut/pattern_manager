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


class Group(object):
    def __init__(self, nm):
        self.name = nm
        self.id = Manager.id
        self.elements = []
        self.par = None
        Manager.id += 1

    def add_subelement(self, e):
        self.elements.append(e)
        e.par = self

    def find_subelement_by_nm(self, nm, e):
        # print e.name

        if not e:
            return None

        if e.name == nm:
            return e

        if isinstance(e, Group):
            for sub in e.elements:
                return self.find_subelement_by_nm(nm, sub)

    @property
    def element_cnt(self):
        len(self.elements)
