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

from bidict import bidict


class Manager(object):
    """This class contains and manages elements of various types.
    
    :param name: The unique name of this manager
    :type name: str
    :param elements: Elements to add to manager, defaults to []
    :type elements: list, optional
    """
    
    def __init__(self, name, elements=[]):
        self.name = name
        self._cur_index = 0
        self.iterator = 0
        self.finished = False
        self.elements = bidict()
        self.active = False
        self.parent = None
        
        for e in elements:
            self.add_element(e)

    def add_element(self, element):
        """Adds an element to the dictionary of elements within the manager.
        
        :param element: An element
        :type element: Object
        """
        
        if hasattr(element, 'parent'):
            element.parent = self
        
        self.elements[self._cur_index] = element
        self._cur_index += 1

    def remove_element(self, index):
        """Removes an element from the dictionary of elements within the manager.
        
        :param index: The index/key of the element to be removed
        :type index: int
        :return: Returns True if successfully removed, otherwise False
        :rtype: bool
        """

        try:
            del self.elements[index]
            return True
        except KeyError:
            return False

    def pop_element(self, index):
        """Pops an element from the dictionary of elements within the manager.
        
        :param index: The index/key of the element to be popped
        :type index: int
        :return: Returns the popped element if successful, otherwise False
        :rtype: Object, False
        """

        try:
            self.elements.pop(index)
        except KeyError:
            return False
   
    def get_element(self, index):
        """Retrieves an element from the dictionary of elements within the manager.
        
        :param index: The index/key of the element to be retrieved
        :type index: int
        :return: Returns the requested element if successful, otherwise False
        :rtype: Object, False
        """

        try:
            e = self.elements[index]
            return e
        except KeyError:
            return False

    def get_element_index(self, element):
        """Retrieves the element index from the element instance.
        
        :param element: An object from which to retrieve the index
        :type element: Object
        :return: Returns the requested index if successful, otherwise False
        :rtype: Object, False
        """

        try:
            return self.elements.inverse[element]
        except KeyError:
            return False

    def get_element_index_by_name(self, name):
        """Retrieves the index of an element by the name of the element.
        
        :param name: A string specififying the name of the element of the index to be retrieved
        :type name: str
        :return: An index of the element
        :rtype: int
        """

        for k in self.elements:
            if self.elements[k].name == name:
                return k

    def get_current_element(self):
        """Retrieves the element currently iterated to.
        
        :return: Returns internal iterator and the currently active element if successful, otherwise False
        :rtype: tuple with iterator, Object, or False if failed
        """

        try:
            (i, e) = self.iterator, self.elements[self.iterator]
            return (i, e)
        except KeyError:
            return False

    def get_next_element(self):
        """Retrieves the next element to become iterated to.
        
        :return: Returns the next element to become active if successful, otherwise False
        :rtype: Object, False
        """

        next_i = self.iterator + 1
        if next_i < self.element_count:
            (i, e) = next_i, self.elements[next_i]
            return (i, e)
        else:
            return None

    def iterate(self):
        """Increases the iterator of the elements.
        
        :return: Returns the iterator increased to if successful. If iteration is finished, False
        :rtype: int, False
        """
        
        next_i = self.iterator + 1
        if next_i < self.element_count:
            self.iterator += 1
        else:
            self.finished = True
            self.active = False

            parent = self.parent
            if not parent is None:
                parent_next = parent.get_next_element()
                if not parent_next is None:
                    parent.active = parent_next[1].active

                parent.iterate()

            return False
        
        return next_i

    def group_elements(self, indices, name):
        """Groups elements within a new manager places in the dictionary of elements.
        
        :param indices: Indices of the elements to be grouped
        :type indices: int
        :param name: The name of the newly created manager
        :type name: str
        :return: Returns True if the creation was successful, otherwise False
        :rtype: bool
        """

        manager = Manager(name)

        for i in indices:
            if self.get_element(i) is False:
                return False
            
            manager.add_element(self.elements.pop(i))
        
        self.add_element(manager)

        return True

    def get_element_type(self, index):
        """Retrieves the type of the element at the specified index.
        
        :param index: The indix of the element of which the type is found
        :type index: int
        :return: Returns the class name of the specified element
        :rtype: str
        """
        return self.get_element(index).__class__.__name__

    def reset(self):
        """Resets the iterator of the manager and sets finished to False
        """

        self.iterator = 0
        self.finished = False

    def sorted_indices(self):
        """Retrieves a sorted list of the keys of the elements.
        
        :return: A sorted list of keys
        :rtype: list
        """

        return sorted(self.elements.keys())

    @property
    def element_count(self):
        """The number of elements within the dictionary of elements of the manager.
        
        :type: int
        """
        
        return len(self.elements)

    # @property
    # def element_finished(self, index):
    #     """Returns the finished status of the element at the specified index of the elements
        
    #     :param index: The index of the element
    #     :type index: int
    #     :return: Returns the status of the specified element
    #     :rtype: bool
    #     """

    #     if index < self.iterator:
    #         return True
    #     else:
    #         return False

    # @property
    # def active_element(self):
    #     """The currently active element of the manager.
        
    #     :type: Object or None
    #     """

    #     if not self.finished:
    #         return self.get_current_element()
    #     else:
    #         return None
