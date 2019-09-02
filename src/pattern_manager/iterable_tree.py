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


class IterableTree(object):
    def __init__(self):
        self.children = {}

    def __iter__(self):
        self.i = 0

        return self

    def next(self):
        if self.i < len(self.children):
            x = self.i
            self.i += 1

            return x
        else:
            raise StopIteration

    def add_child(self, chld):
        i = len(self.children)
        self.children[i] = chld

    def remove_child(self, chld_i):
        try:
            del self.children[chld_i]
        except KeyError:
            print 'Error: child index does not exist'

    def pop_child(self, chld_i):
        try:
            c = self.children.pop(chld_i)

            return c
        except KeyError:
            print 'Error: child index does not exist'

    def get_child(self, chld_i):
        try:
            c = self.children[chld_i]

            return c
        except KeyError:
            print 'Error: child index does not exist'

    def get_child_by_name(self, nm):
        for c in self.children.values():
            if c.name != nm:
                continue

            return c
