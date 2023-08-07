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

import geometry_msgs.msg as gm_msg

from collections import OrderedDict


class XForm(gm_msg.Transform):
    """
    This class describes a tree of transforms and each nodes relation to other nodes

    :param parent: The parent XForm of this XForm object
    :type parent: XForm, None
    :param name: The name of this XForm
    :type name: str
    :param ref_frame: The name of the reference frame of the XForm
    :type ref_frame: str, optional
    """

    #: The number of XForms created
    count = 0

    def __init__(self, parent, name, ref_frame=None):
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

        if self.parent:
            self.parent.children[id(self)] = self

            if not ref_frame:
                self.ref_frame = self.parent.name

    def add_node(self, chld):
        """
        This function adds a child to the XForm and assigns itself as the childs parent

        :param chld: The child object to be added
        :type chld: XForm
        """

        self.children[id(chld)] = chld

        if chld.parent:
            del chld.parent.children[id(chld)]

        chld.parent = self

    def set_active(self, actv):
        """
        This function sets the XForm and all of its descendants active/inactive

        :param actv: This value determines whether to set the XForms active or inactive
        :type actv: bool
        """

        if len(self.children) > 0:
            for c in self.children.values():
                c.set_active(actv)
        elif self.parent:
            self.active = actv

    def get_current_node(self, root=None):
        """
        This function retrieves the currently first active XForm in the tree

        :return: Returns the first active XForm in the tree
        :rtype: XForm
        """

        if not root:
            root = self

        lst = self.get_active_nodes(root)

        if len(lst) > 0:
            return lst[0]
        else:
            return None

    def get_active_nodes(self, root=None):
        """
        This function retrieves all nodes which are set active

        :param root: The XForm to begin the tree-search from
        :type root: XForm, optional
        :return: Returns all XForms which are currently set to active within the tree
        :rtype: list
        """

        if not root:
            root = self

        lst = []

        if root.active:
            if len(root.children) == 0:
                lst.append(root)

        for n in root.children.values():
            lst.extend(self.get_active_nodes(n))

        return lst

    def recursive_remove_node(self, id_, root=None):
        """
        This function initiates the removal of an XForm (tree-node) and all of its descendants

        :param id_: The ID of the XForm to begin the recursive removal from
        :type id_: int
        """

        if not root:
            root = self

        self._recursive_remove_node(id_)

        node = self.get_node(id_, root)

        if node.parent:
            del node.parent.children[id_]
        else:
            del node

    def _recursive_remove_node(self, id_, root=None):
        """
        This function recursively removes an XForm (tree-node) and all of its descendants

        :param id_: The ID of the XForm to begin the recursive removal from
        :type id_: int
        """

        if not root:
            root = self

        for k, v in self.get_node(id_, root).children.copy().items():
            self._recursive_remove_node(k)
            del self.get_node(id_, root).children[k]

    def get_nodes(self, root=None):
        """
        This function retrieves all XForm objects of the tree

        :param root: The XForm to begin the tree-search from
        :type root: XForm, optional
        :return: Returns all XForms within the tree
        :rtype: list
        """

        if not root:
            root = self

        lst = [root]

        for n in root.children.values():
            lst.extend(self.get_nodes(n))

        return lst

    def get_node(self, id_, root=None):
        """
        This function returns an XForm if one matches the ID

        :param id_: The ID of the XForm object to be retrieved
        :type id_: int
        :param root: The XForm to begin the tree-search from
        :type root: XForm, optional
        :return: Returns the requested XForm if there is a match, else None
        :rtype: XForm
        """

        if not root:
            root = self

        if id(root) == id_:
            return root
        else:
            res = None
            for c in root.children.values():
                res = self.get_node(id_, c)

                if res:
                    break

            return res

    def iterate(self):
        """
        This function iterates the list of active XForms, setting the current XForm inactive

        :return: `True` if successful, else `False`
        :rtype: bool
        """

        actv = self.get_current_node()

        if actv:
            actv.active = False

            return True
        else:
            return False

    def clear(self):
        """
        This function resets the tree from this node, effectively removin all descendant nodes
        """
        self.recursive_remove_node(id(self))

    def to_dict(self, root=None, dict_=None):
        """
        This function creates a dictionary object from the XForm tree

        :param root: The XForm to begin the dictionary from
        :type root: XForm, optional
        :param dict_: The dictionary object to populate
        :type dict_: dict, optional
        :return: Returns a dictionary of the XForm tree
        :rtype: dict
        """

        if not root:
            root = self

        if not dict_:
            dict_ = {}

        dict_[root.name] = {
                'ref_frame': root.ref_frame,
                'translation': [root.translation.x, root.translation.y, root.translation.z],
                'rotation': [root.rotation.x, root.rotation.y, root.rotation.z, root.rotation.w]
        }

        for k, v, in root.children.items():
            self.to_dict(v, dict_[root.name])

        return dict_

    def from_dict(self, dict_, root=None):
        """
        This function creates a XForm tree from a dictionary object

        :param dict_: The dictionary to create the XForm tree from
        :type dict_: dict
        :param root: The current root XForm of the tree
        :type root: XForm
        """

        if not root:
            root = self
            dict_ = dict_[list(dict_)[0]]

        for k, v in dict_.items():

            if isinstance(v, dict):
                child = XForm(root, name=k, ref_frame=dict_[k]['ref_frame'])
                child.translation = gm_msg.Vector3(
                    x=dict_[k]['translation'][0],
                    y=dict_[k]['translation'][1],
                    z=dict_[k]['translation'][2]
                )
                child.rotation = gm_msg.Quaternion(
                    x=dict_[k]['rotation'][0],
                    y=dict_[k]['rotation'][1],
                    z=dict_[k]['rotation'][2],
                    w=dict_[k]['rotation'][3]
                )

                self.from_dict(dict_[k], child)
