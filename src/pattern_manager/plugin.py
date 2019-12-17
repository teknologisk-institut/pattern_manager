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

import os, sys, inspect
import pkgutil, importlib


class Plugin(object):
    """
    This class is the base plugin object for pattern plugins

    :param parent: An XForm parent object under which to create the XForm pattern
    :type parent: XForm
    """

    def __init__(self, parent):
        self.parent = parent

    def generate(self):
        """
        This abstract method is implemented in each plugin and is responsible for generating the
        specific pattern of XForm objects
        """

        raise NotImplementedError


class PluginLoader(object):

    def __init__(self, module):
        self.plugins = {}
        self.module = module

        self.load_patterns()

    def load_patterns(self):
        pkg = importlib.import_module(self.module)

        self.plugins = {}
        for _, name, ispkg in pkgutil.iter_modules(pkg.__path__, pkg.__name__ + '.'):

            if ispkg:
                continue

            plugin_module = importlib.import_module(name)
            cls_members = inspect.getmembers(plugin_module, inspect.isclass)

            for _, cls in cls_members:

                if issubclass(cls, Plugin) and cls is not Plugin:
                    self.plugins[name.split('.')[-1]] = cls
