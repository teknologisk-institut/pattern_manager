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

# Author: Mikkel Rath Hansen

from pluginlib import PluginLoader
from pattern_manager.patterns import pattern_base
from pattern_manager.collection import Manager
from copy import copy

import pattern_manager.examples as ex


class PatternFactory:
    """A factory for creating patterns.

    Offers methods for generating internal model of patterns, based on pattern parameters.
    """

    def __init__(self):
        self._patterns = {}

    def register_pattern_type(self, pattern_type, pattern):
        """Registers a new pattern type.
        
        :param pattern_type: a string defining the pattern type to be retrieved
        :type pattern_type: str
        :param pattern: A Pattern object
        :type pattern: Pattern
        """

        self._patterns[pattern_type] = pattern

    def get_pattern(self, pattern_type, base_params, pattern_params):
        """Retrieves a pattern object of specified type.
        
        :param pattern_type: A string defining the pattern type
        :type pattern_type: str
        :param base_params: A dictionary defining parameters for the Pattern base class
        :type base_params: dict
        :param pattern_params: A dictionary defining parameters for the pattern class
        :type pattern_params: dict
        :raises ValueError: Raised if the requested pattern type is None
        :return: A specific pattern type inheriting from Pattern base class
        :rtype: Pattern
        """

        pattern = self._patterns.get(pattern_type)

        if not pattern:
            raise ValueError(pattern_type)

        return pattern(base_params, **pattern_params)


class API(object):
    """The interface class for the pattern manager package.

    This class defines the interface which affords functionalities of the pattern manager package.
    
    :param pattern_dicts: a list of pattern dictionaries, defaults to []
    :type pattern_dicts: list, optional
    :raises Exception: Raised if an instance of API already exists
    """

    _instance = None

    @staticmethod
    def getInstance():
        """Fetches the single instance of the API class.
        
        :return: The single instance of the API class
        :rtype: API
        """

        if API._instance is not None:
            API()
        return API._instance

    def __init__(self, pattern_dicts=[]):
        if API._instance is not None:
            raise Exception("Instance already exists - this class is a singleton")
        else:
            API._instance = self

        self._factory = PatternFactory()
        self._loader = PluginLoader(group='patterns')
        self._load_pattern_types()
        self.manager = Manager("group_0")
        self.manager.active = True
        self.patterns = {}
        self._pattern_index = 0

        for d in pattern_dicts:
            self.create_pattern_from_dict(
                d['pattern_params'],
                d['base_params']
            )

    def _load_pattern_types(self):
        """This function loads patterns from plugin classes.
        """

        for k in self._loader.plugins['pattern'].keys():
            self._factory.register_pattern_type(
                k, self._loader.get_plugin('pattern', k))

    def create_pattern_from_dict(self, pattern_params, base_params):
        """This function creates and stores a pattern from a dictionary description.
        
        :param pattern_params: Parameters required for the instantiation of a pattern
        :type pattern_params: dict
        :param base_params: Parameters required for the pattern base class
        :type base_params: dict
        """

        pattern_type = pattern_params.pop('pattern_type')
        pattern = self._factory.get_pattern(
            pattern_type, base_params, pattern_params)
        self.manager.add_element(pattern)

        self.patterns[self._pattern_index] = pattern
        self._pattern_index += 1

    def get_active_leaf_manager(self, element=None):
        """Retrieves the active manager lowest in the manager tree from the specified start element.
        
        :param element: The element to begin search from, defaults to None
        :type element: Manager, optional
        :return: The active manager lowest in the tree
        :rtype: Manager
        """

        if element is None:
            element = self.manager

        mans = []
        for e in element.elements.values():
            if isinstance(e, Manager) and e.active:
                mans.append(e)

        if len(mans) == 0:
            return element
        else:
            for m in mans:
                return self.get_active_leaf_manager(m)

    def get_active_root_manager(self):
        """Retrieves the active root of the active leaf manager
        
        :return: The active root manager
        :rtype: Manager
        """

        try:
            man = self.get_active_leaf_manager()
        except:
            print "Error: No active manager!"
            return None

        while man.parent is not None and man.parent.active:
            man = man.parent
        
        return man

    def get_successors_from_manager_tree(self, arr, element=None):
        """Retrieves all elements under the specified element in the tree.
        
        :param arr: An array to place the elements in
        :type arr: list
        :param element: The element to start the search from, defaults to None
        :type element: Manager or Pattern, optional
        """

        if element is None:
            element = self.manager

        arr.append(element)

        if isinstance(element, Manager):
            for e in element.elements.values():
                self.get_successors_from_manager_tree(arr, e)

    def get_element_by_name(self, name, element=None):
        """Retrieves an element with the specified name.
        
        :param name: The name of the requested element
        :type name: str
        :param element: The element in the tree to begin the search from, defaults to None
        :type element: Manager or Pattern, optional
        :return: Return the element with the specified name
        :rtype: Manager or Pattern
        """

        if element is None:
            element = self.manager

        elements = []
        self.get_successors_from_manager_tree(elements, element)

        for e in elements:
            if e.name == name:
                return e

        return None

    def reset_element_tree(self, element):
        """Reset all managers and/or patterns from (and including) the specified start element.
        
        :param element: The element in the tree to begin the search from
        :type element: Manager or Pattern
        """
        
        elements = []
        self.get_successors_from_manager_tree(elements, element)

        for e in elements:
            e.reset()

    def iterate_active(self):
        """Iterates through the active instances of the manager tree.
        
        :return: Returns the value of the currently active managers iterate function
        :rtype: int, bool
        """
        
        return self.get_active_leaf_manager().iterate()

    def set_active_manager(self, element, active=True):
        """Sets active the specified manager and all of its ancestors.
        
        :param element: The manager to set active
        :type element: Manager
        :param active: Whether the manager and ancestors should be activated or deactivated, defaults to True
        :type active: bool, optional
        :return: True if successful, else False
        :rtype: bool
        """

        if not isinstance(element, Manager):
            print "Error: element is not a manager!"
            return False

        if active:
            self.set_active_manager(self.get_active_leaf_manager(), False)

        while element is not None:
            element.active = active
            element = element.parent

        return True


if __name__ == '__main__':
    ds = [ex.linear_d, ex.linear_d2, ex.rect_d, ex.scatter_d, ex.circle_d]

    api = API(ds)
    man = api.manager

    # group patterns and group managers
    man.group_elements([0, 1], "group_1")
    man.group_elements([2, 3, 4], "group_2")
    man.group_elements([5, 6], "group_3")

    # manually set active elements
    api.set_active_manager(man.elements[7].elements[0])

    b_man = api.get_element_by_name("group_2")
    print "element found by name, 'group_2': {}".format(b_man.name)

    a_man = api.get_active_leaf_manager()
    print "active leaf manager: {}".format(a_man.name)

    c_man = api.get_active_root_manager()
    print "active root manager: {}".format(c_man.name)

    arr = []
    api.get_successors_from_manager_tree(arr, None)

    print ""

    print "all nodes from root manager tree:"
    for a in arr:
        print "    {}".format(a.name)

    print ""

    man3 = api.get_element_by_name("group_3")
    man3.allow_iterate = False

    for i in range(3):
        print "Current element (in {}): {}".format(a_man.name, a_man.get_current_element()[1].name)
        api.iterate_active()
        a_man = api.get_active_leaf_manager()

    print ""

    print "re-setting all elements under root manager: "
    api.reset_element_tree(api.get_element_by_name("group_0"))
    print "    done"

    print ""

    man1 = api.get_element_by_name("group_1")
    print "Current element (in {}): {}\n".format(man1.name, man1.get_current_element()[1].name)
