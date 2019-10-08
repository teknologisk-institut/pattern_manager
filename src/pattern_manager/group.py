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

from src.pattern_manager.manager import Manager


class Group(object):

    _instances = {}

    def __init__(self, nm, par=None):
        self.name = nm
        self.type = 'Group'
        self.parent = par
        self.group_type = None
        self.children = []

        if self.parent:
            self.parent.add_node(self)

        Group._instances[self.name] = self

        Manager.register_id(id(self))

    def add_child(self, chld):

        if not self.group_type:
            self.group_type = chld.type
        elif self.group_type != chld.type:
            print "Warning: only objects of type {} can be added".format(self.group_type)

            return False

        self.children.append(chld)
        chld.parent = self

        return True

    @staticmethod
    def get_group_by_name(nm):
        try:
            grp = Group._instances[nm]

            return grp
        except KeyError:
            print "Error: There exists no container with the name: {}".format(nm)

            return

    @staticmethod
    def print_tree(cont, prefix=''):
        print prefix + 'id: {}, name: {}'.format(id(cont), cont.name)

        if not isinstance(cont, Group):
            return

        prefix += '\t'
        for c in cont.children:
            Group.print_tree(c, prefix)
