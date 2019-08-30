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

from ..container import Container


class Group(Container):

    _instances = {}

    def __init__(self, nm, par=None):
        super(Group, self).__init__(nm)

        self.group_type = None

        if self.parent:
            self.parent.add_child(self)

        Group._instances[self.name] = self

    @property
    def type(self):
        return "Group"

    def add_child(self, chld):

        if not self.group_type:
            self.group_type = chld.type
        elif self.group_type != chld.group_type:
            print "Warning: only objects of type {} can be added".format(self.group_type)

            return

        super(Group, self).add_child(chld)
