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

from abc import ABCMeta, abstractproperty


class Container(object):
    __metaclass__ = ABCMeta

    _instances = {}

    def __init__(self, nm):
        self.name = nm
        self.parent = None
        self.children = []

        Container._instances[self.name] = self

    @abstractproperty
    def type(self):
        raise NotImplementedError()

    def add_child(self, chld):
        self.children.append(chld)
        chld.parent = self

        return True

    @staticmethod
    def get_container_by_name(nm):
        try:
            cont = Container._instances[nm]

            return cont
        except KeyError:
            print "Error: There exists no container with the name: {}".format(nm)

            return

    @staticmethod
    def print_tree(cont):
        print id(cont), cont.name

        if not isinstance(cont, Container):
            return

        for c in cont.children:
            Container.print_tree(c)

    def child_count(self):
        len(self.children)
