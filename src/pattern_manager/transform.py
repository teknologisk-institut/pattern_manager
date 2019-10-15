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

import rospy
import geometry_msgs.msg as gm_msg


class Transform(gm_msg.Transform):

    root = None

    def __init__(self, nm, par, ref_frame=None):
        super(Transform, self).__init__()

        self.name = nm
        self.parent = par
        self.active = False
        self.i = 0
        self.children = {}
        self.ref_frame = ref_frame

        if not Transform.root:
            Transform.root = self

        if not ref_frame:
            self.ref_frame = self.parent.ref_frame

        if self.parent:
            self.parent.add_node(self)

    def add_node(self, chld):
        self.children[id(chld)] = chld
        chld.parent = self

    def set_active(self, actv):
        self.active = actv

        for c in self.children.values():
            if len(c.children) == 0:
                c.set_active(actv)

    @staticmethod
    def get_active_nodes(root=None):
        lst = []

        if not root:
            root = Transform.root

        if root.active:
            lst.append(root)

        for n in root.children.values():
            lst.extend(Transform.get_active_nodes(n))

        return lst

    @staticmethod
    def remove_node(id_):
        if not Transform.get_node(id_).parent:
            rospy.logwarn("Removing the root transform is not allowed")

            return

        Transform._remove_node(id_)
        del Transform.get_node(id_).parent.children[id_]

    @staticmethod
    def _remove_node(id_):
        for k, v in Transform.get_node(id_).children.items():
            Transform._remove_node(k)
            del Transform.get_node(id_).children[k]

    @staticmethod
    def get_nodes(root=None):
        lst = []

        if not root:
            root = Transform.root

        lst.append(root)

        for n in root.children.values():
            lst.extend(Transform.get_nodes(n))

        return lst

    @staticmethod
    def get_node(id_, root=None):

        if not root:
            root = Transform.root

        if id(root) == id_:
            return root
        else:
            res = None
            for c in root.children.values():
                res = Transform.get_node(id_, c)
                if res:
                    break

            return res

    @staticmethod
    def iterate():
        l_actv = Transform.get_active_nodes()

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
