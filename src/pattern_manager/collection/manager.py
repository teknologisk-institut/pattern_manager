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

from abc import ABCMeta


class Manager(object):
    """The Manager class is responsible for keeping track of the iteration \
        of all groups and patterns and whether a group or pattern has finished \
            and/or is active.
    """

    __metaclass__ = ABCMeta

    i = {}
    finished = {}
    active = {}
    # TODO: iteration order for each group -- self.iter_ordr = []

    @staticmethod
    def register_id(id):
        """This function lets a group or pattern register themselves with the Manager.
        
        :param id: The element ID
        :type id: str
        """

        Manager.i[id] = 0
        Manager.finished[id] = False
        Manager.active[id] = False

    @staticmethod
    def iterate(e):
        """This function iterates the supplied pattern or group.
        
        :param e: A group or pattern to be iterated
        :type e: Group, Pattern
        :return: True if input element is iterable, else False
        :rtype: bool
        """

        nxt_i = Manager.i[id(e)] + 1

        count = 0
        if e.typ == "Pattern":
            count = len(e.tfs)
        elif e.typ == "Group":
            count = len(e.chldrn)

        if not nxt_i < count:
            Manager.set_finished(id(e), True)
            Manager.set_active(id(e), False)

            if e.par:
                Manager.iterate(e.par)

            return False

        Manager.i[id(e)] = nxt_i

        return True

    @staticmethod
    def reset_element(id):
        """This function resets the iteration, finished, and active state of \
            a given element id.
        
        :param id: The ID of the element to reset.
        :type id: str
        """

        Manager.i[id] = 0
        Manager.finished[id] = False

    @staticmethod
    def set_active(id, actv):
        """This function sets the active value of an element id.
        
        :param id: The ID of the element to set active
        :type id: str
        :param actv: Specifies whether the active value of the element should \
            be True or False.
        :type actv: bool
        """

        Manager.active[id] = actv

    @staticmethod
    def set_finished(id, fin):
        """This function sets the finished value of an element id.
        
        :param id: The ID og the element to set finished.
        :type id: str
        :param fin: Specifies whether the finished value of the element should \
            be True or False
        :type fin: bool
        """

        Manager.finished[id] = fin

    @staticmethod
    def get_active_group(root):
        """This function returns the currently active leaf group.
        
        :param root: The root Group to begin the search from.
        :type root: Group
        :return: Returns the currently active group.
        :rtype: Group
        """

        for g in root.chldrn:
            if g.typ == "Group" and Manager.active[id(g)]:
                return Manager.get_active_group(g)

        return root

    @staticmethod
    def set_active_group(grp):
        """This function sets the active value of a group id, as well as \
            all of the groups parents and subelements.
        
        :param grp: The group to be set active
        :type grp: Group
        """

        Manager.set_active(id(grp), True)
        Manager.set_active_subs(grp, True)
        Manager.set_active_supers(grp, True)

    @staticmethod
    def get_active_pattern(root):
        """This function returns the currently active pattern.
        
        :param root: The group to start the search from.
        :type root: Group
        :return: Returns the currently active pattern.
        :rtype: Pattern
        """

        actv_grp = Manager.get_active_group(root)
        
        if not actv_grp.g_typ == "GOP" or actv_grp.child_cnt == 0:
            return None

        i = Manager.i[id(actv_grp)]

        return actv_grp.chldrn[i]

    @staticmethod
    def set_active_pattern(pat):
        """This function sets the active value of a pattern id, as well as \
            all of the patterns parents.
        
        :param pat: The pattern to set active.
        :type pat: Pattern
        """

        Manager.set_active(id(pat), True)
        Manager.set_active_supers(pat, True)

    @staticmethod
    def set_active_supers(e, actv):
        """This function sets the active value of an elements parents.
        
        :param e: The element which parents are to be set active.
        :type e: Group, Pattern
        :param actv: Specifies whether the active value should be True or False.
        :type actv: bool
        :return: Returns True of successful.
        :rtype: bool
        """

        while e.par:
            Manager.active[id(e.par)] = actv
            e = e.par

        return True

    @staticmethod
    def set_active_subs(e, actv):
        """This function sets the active value of an elements subelements.
        
        :param e: The element which subelements are to be set active.
        :type e: Group, Pattern
        :param actv: Specifies whether the active value should be True or False.
        :type actv: bool
        """

        for sub in e.chldrn:
            Manager.active[id(sub)] = actv

            if e.g_typ == "GOG":
                Manager.set_active_subs(sub, actv)

    @staticmethod
    def reset_subs(grp):
        """The function calls the reset_element function for all subelements of a group.
        
        :param grp: The group which subelements are to be reset.
        :type grp: Group
        """

        for sub in grp.chldrn:
            Manager.reset_element(id(sub))

            if grp.g_typ == "GOG":
                Manager.reset_subs(sub)

    @staticmethod
    def print_active_subs(grp):
        """This function prints the id and name of all active subelements of a group.
        
        :param grp: The group which active subelements should be printed.
        :type grp: Group
        """

        if grp.typ == "Group":
            for sub in grp.chldrn:
                if Manager.active[id(sub)]:
                    print id(sub), sub.nm
                    Manager.print_active_subs(sub)