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

""" Factory for creating the specified pattern object """
class PatternFactory:
    def __init__(self):
        self._patterns = {}

    def register_pattern_type(self, pattern_type, pattern):
        self._patterns[pattern_type] = pattern

    def get_pattern(self, pattern_type, base_params, pattern_params):
        pattern = self._patterns.get(pattern_type)

        if not pattern:
            raise ValueError(pattern_type)

        return pattern(base_params, **pattern_params)


""" This singleton is the interface of the pattern manager """
class API(object):
    __instance = None

    @staticmethod
    def getInstance():
        if API.__instance == None:
            API()

        return API.__instance

    def __init__(self, pattern_dicts=[]):
        if API.__instance != None:
            raise Exception("Instance already exists!")
        else:
            API.__instance = self

        self.__factory = PatternFactory()
        self.__loader = PluginLoader(group='patterns')
        self.__load_pattern_types()
        self.manager = Manager()

        for d in pattern_dicts:
            self.create_pattern_from_dict(
                d['pattern_params'],
                d['base_params']
            )

    """ Loads the different pattern types from plugin classes """
    def __load_pattern_types(self):
        for k in self.__loader.plugins['pattern'].keys():
            self.__factory.register_pattern_type(k, self.__loader.get_plugin('pattern', k))

    """ Creates a pattern from a dictionary specifying the various pattern parameters """
    def create_pattern_from_dict(self, pattern_params, base_params):
        pattern_type = pattern_params.pop('pattern_type')
        pattern = self.__factory.get_pattern(pattern_type, base_params, pattern_params)
        self.manager.add_element(pattern)
        
        return pattern

    def get_active_manager(self, m):
        a = None
        for k in m.elements.keys():
            e = m.get_element(k)
            e_type = m.get_element_type(k)

            if not e_type == "Manager":
                continue
            elif e.active:
                a = e
                break
        
        if a is not None:
            return self.get_active_manager(a)
        else:
            if m is not self.manager:
                return m
            else:
                return a
    
    def iterate(self):
        g = self.get_active_manager(self.manager)
        g.get_current_element().iterate()

    def reset(self, element):
        



if __name__ == '__main__':
    ds = [ex.linear_d, ex.linear_d2, ex.rect_d, ex.scatter_d, ex.circle_d]

    interface = API(ds)
    man = interface.manager
    
    for e in man.elements.keys():
        print "index: {} | type: {}".format(e, man.get_element_type(e))

    man.group_elements([0, 3])
    print ""

    for e in man.elements.keys():
        print "index: {} | type: {}".format(e, man.get_element_type(e))

    print ""

    for e in man.get_element(5).elements.keys():
        print "index: {} | type: {}".format(e, man.get_element(5).get_element_type(e))

    man.set_active(True)
    man.get_element(5).set_active(True)

    g = interface.get_active_manager(man)
    print man.get_element_id(g)