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
    def __init__(self, name, elements=[]):
        self._name = name
        self.cur_index = 0
        self._iterator = 0
        self.finished = False
        self.elements = bidict()
        self._active = False
        
        for e in elements:
            self.add_element(e)

    def add_element(self, element):
        self.elements[self.cur_index] = element
        self.cur_index += 1

    def remove_element(self, index):
        try:
            del self.elements[index]
            return True
        except KeyError:
            return False

    def pop_element(self, index):
        try:
            self.elements.pop(index)
        except KeyError:
            return False
   
    def get_element(self, index):
        try:
            e = self.elements[index]
            return e
        except KeyError:
            return False

    def get_element_index(self, element):
        try:
            return self.elements.inverse[element]
        except KeyError:
            return False

    def get_element_index_by_name(self, name):
        for k in self.elements:
            if self.elements[k].name == name:
                return k

    def get_current_element(self):
        try:
            (i, e) = self._iterator, self.elements[self._iterator]
            return (i, e)
        except KeyError:
            return False

    def get_next_element(self):
        next_i = self._iterator + 1
        if next_i < self.element_count:
            return next_i
        else:
            return False

    def increase_iterator(self):
        next_i = self._iterator + 1
        if next_i < self.element_count:
            self._iterator += 1
        else:
            self.finished = True
            return False
        
        return next_i

    def group_elements(self, indices, name):
        manager = Manager(name)

        for i in indices:
            if self.get_element(i) is False:
                return False
            
            manager.add_element(self.elements.pop(i))
        
        self.add_element(manager)

        return True

    def get_element_type(self, index):
        return self.get_element(index).__class__.__name__

    def reset(self):
        self._iterator = 0
        self.finished = False

    @property
    def element_count(self):
        return len(self.elements)

    @property
    def finished(self):
        return self.finished

    @property
    def element_finished(self, index):
        if index < self._iterator:
            return True
        else:
            return False

    def sorted_indices(self):
        return sorted(self.elements.keys())

    @property
    def active(self):
        return self._active

    @active.setter
    def active(self, active):
        self._active = active

    @property
    def active_element(self):
        if not self.finished:
            return self.get_current_element()
        else:
            return False

    @property
    def iterator(self):
        return self._iterator

    @iterator.setter
    def _iterator(self, i):
        self._iterator = i

    @property
    def name(self):
        return self._name
    
    @name.setter
    def _name(self, name):
        self._name = name
