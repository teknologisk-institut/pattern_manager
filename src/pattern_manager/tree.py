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

from itertools import chain, imap


class Tree(object):

    root = None

    def __init__(self, nm, par=None):
        self.nodes = {}
        self.patterns = {}
        self.name = nm
        self.parent = par
        self.active = False
        self.i = 0
        self.type = 'Tree'

        if not Tree.root:
            Tree.root = self

        if self.parent:
            self.parent.add_node(self)

    def __iter__(self):
        for v in chain(*imap(iter, self.nodes.values())):
            yield v

        if self.active:
            yield self

    def add_node(self, chld):
        self.nodes[id(chld)] = chld
        chld.parent = self

    def add_pattern(self, pat):
        self.patterns[id(pat)] = pat
        pat.parent = self

    def set_active(self, active):
        self.active = active

        if self.parent:
            self.parent.set_active(active)

    @staticmethod
    def get_active_nodes():
        return list(iter(Tree.root))

    @staticmethod
    def get_node(id_, node=None):

        if not node:
            node = Tree.root

        if id(node) == id_:
            return node
        else:
            res = None
            for c in node.nodes.values():
                res = Tree.get_node(id_, c)
                if res:
                    break

            return res

    # @staticmethod
    # def get_pattern(id_, node=None):
    #     if not node:
    #         node = Tree.root
    #
    #     if id(node) == id_:
    #         return node
    #     else:
    #         res = None
    #         for c in node.nodes.values():
    #             res = Tree.get_node(id_, c)
    #             if res:
    #                 break
    #
    #         return res

    @staticmethod
    def iterate():
        l_actv = Tree.get_active_nodes()

        if len(l_actv) == 0:
            return False

        leaf = l_actv[0] if len(l_actv) > 0 else None
        parent = l_actv[1] if len(l_actv) > 1 else None

        nxt_i = leaf.i + 1

        if not nxt_i < len(leaf.get_children()):
            leaf.active = False

            if parent:
                parent.iterate()

            return False

        leaf.i = nxt_i

        return True
