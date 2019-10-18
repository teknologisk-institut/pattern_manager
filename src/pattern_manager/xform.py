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


class XForm(gm_msg.Transform):

    root = None

    def __init__(self, nm, par, ref_frame=None):
        super(XForm, self).__init__()

        self.name = nm
        self.parent = par
        self.active = False
        self.i = 0
        self.children = {}
        self.ref_frame = ref_frame

        if not XForm.root:
            XForm.root = self

        if self.parent:
            self.parent.add_node(self)

            if not ref_frame:
                self.ref_frame = self.parent.ref_frame

    def add_node(self, chld):
        self.children[id(chld)] = chld
        chld.parent = self

    def set_active(self, actv):
        self.active = actv

        if self.parent:
            self.parent.set_active(actv)

    @staticmethod
    def get_current_node():
        lst = XForm.get_active_nodes()

        if len(lst) > 0:
            lst.reverse()

            return lst[0]
        else:
            return None

    @staticmethod
    def get_active_nodes(root=None):
        lst = []

        if not root:
            root = XForm.root

        if root.active:
            lst.append(root)

        for n in root.children.values():
            lst.extend(XForm.get_active_nodes(n))

        return lst

    @staticmethod
    def remove_node(id_):
        if not XForm.get_node(id_).parent:
            rospy.logwarn("Removing the root transform is not allowed")

            return

        XForm._remove_node(id_)
        del XForm.get_node(id_).parent.children[id_]

    @staticmethod
    def _remove_node(id_):
        for k, v in XForm.get_node(id_).children.items():
            XForm._remove_node(k)
            del XForm.get_node(id_).children[k]

    @staticmethod
    def get_nodes(root=None):
        lst = []

        if not root:
            root = XForm.root

        lst.append(root)

        for n in root.children.values():
            lst.extend(XForm.get_nodes(n))

        return lst

    @staticmethod
    def get_node(id_, root=None):

        if not root:
            root = XForm.root

        if id(root) == id_:
            return root
        else:
            res = None
            for c in root.children.values():
                res = XForm.get_node(id_, c)
                if res:
                    break

            return res

    @staticmethod
    def iterate():
        actv = XForm.get_current_node()

        if actv:
            actv.active = False

            return True
        else:
            return False
