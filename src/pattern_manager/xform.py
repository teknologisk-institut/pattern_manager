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

from collections import OrderedDict


class XForm(gm_msg.Transform):

    root = None
    count = 0

    def __init__(self, parent, name=None, ref_frame=None):
        super(XForm, self).__init__(
            translation=gm_msg.Vector3(x=0.0, y=0.0, z=0.0),
            rotation=gm_msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )

        self.parent = parent
        self.name = name
        self.active = False
        self.i = 0
        self.children = OrderedDict()
        self.ref_frame = ref_frame
        self.number = XForm.count

        XForm.count += 1

        if not name:
            self.name = 'tf_' + str(self.number)

        if not XForm.root:
            XForm.root = self

        if self.parent:
            self.parent.add_node(self)

            if not ref_frame:
                self.ref_frame = self.parent.name

    def add_node(self, chld):
        self.children[id(chld)] = chld
        chld.parent = self

    def set_active(self, actv):

        if len(self.children) > 0:
            for c in self.children.values():
                c.set_active(actv)
                rospy.logwarn('%s | %s | %s' % ('tf_%d' % c.number, c.active, actv))
        elif self.parent:
            self.active = actv

    @staticmethod
    def get_current_node():
        lst = XForm.get_active_nodes()

        if len(lst) > 0:
            return lst[0]
        else:
            return None

    @staticmethod
    def get_active_nodes(root=None):
        lst = []

        if not root:
            root = XForm.root

        if root.active:
            if len(root.children) == 0:
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
