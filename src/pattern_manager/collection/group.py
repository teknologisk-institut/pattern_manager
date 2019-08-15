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
import Queue

class GType(Enum):
    """An enum specifying two types of groups.
    """

    GOG = 1
    GOP = 2


class Group(object):
    """This class functions as a data tree for grouping groups and patterns.
    """

    def __init__(self, g_typ, nm, par=None):
        """The class constructor.
        
        :param g_typ: The type of group to be used.
        :type g_typ: GType
        :param nm: The name of the group.
        :type nm: str
        :param par: The parent of the group, defaults to None
        :type par: Group, optional
        """

        self.g_typ = g_typ.name
        self.typ = self.__class__.__name__
        self.nm = nm
        self.chldrn = []
        self.par = par

        if self.par:
            par.add_child(self)

        Manager.register_id(id(self))

    def add_child(self, chld):
        """This function adds a child element to the chldrn list of object.
        
        :param chld: The element to be added to the list of children.
        :type chld: Group, Pattern
        :return: True if successful, else False
        :rtype: bool
        """

        if (self.g_typ == "GOP" and chld.typ == "Group") or \
                (self.g_typ == "GOG" and chld.typ == "Pattern"):
            return False

        self.chldrn.append(chld)
        chld.par = self

        return True
    
    @staticmethod
    def get_sub_by_name(nm, root):
        """This function retrieves a subelement by name.
        
        :param nm: The name of the subelement to be retrieved.
        :type nm: str
        :param root: The root element to initiate the search from.
        :type root: Group
        :return: The element which has the specified name.
        :rtype: Group, Pattern
        """

        q = Queue.Queue()
        q.put(root)

        while not q.empty():
            node = q.queue[0]
            if node.nm == nm:
                return node

            q.get()

            for sub in node.chldrn:
                q.put(sub)

        return None

    def get_sub_groups(self):
        q = Queue.Queue()
        q.put(self)

        subs = []
        while not q.empty():
            node = q.queue[0]

            q.get()

            for sub in node.chldrn:
                if sub.typ == 'Group':
                    subs.append((id(sub), id(node)))
                    q.put(sub)

        return subs

    @staticmethod
    def print_tree(chld):
        """This function prints the id and name of the group and all of \
            its subelements.
        
        :param chld: The current element being printed.
        :type chld: Group, Pattern
        """

        print id(chld), chld.nm

        if chld.typ == "Group":
            for sub in chld.chldrn:
                Group.print_tree(sub)

    def child_cnt(self):
        """This function returns the count of elements in the list of \
            children.
        """
        
        len(self.chldrn)
