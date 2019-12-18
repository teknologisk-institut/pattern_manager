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

import inspect
import pkgutil
import importlib


class Plugin(object):
    """
    This class is the base class for plugins
    """

    def __init__(self):
        pass

    def process(self):
        """
        This method is implemented in each Plugin subclass
        """

        raise NotImplementedError


class PluginLoader(object):
    """
    This class is responsible for loading plugins from a specified package

    :param package: The name of the package to load plugins from
    :type package: str
    """

    def __init__(self, package):
        self.plugins = {}
        self.package = package

        self.load_plugins()

    def load_plugins(self):
        """
        This function walks a package's modules to find and load plugins
        """

        pkg = importlib.import_module(self.package)

        self.plugins = {}
        for _, name, ispkg in pkgutil.iter_modules(pkg.__path__, pkg.__name__ + '.'):

            if ispkg:
                continue

            plugin_module = importlib.import_module(name)
            cls_members = inspect.getmembers(plugin_module, inspect.isclass)

            for _, cls in cls_members:

                if not issubclass(cls, Plugin) or cls is Plugin:
                    continue

                self.plugins[name.split('.')[-1]] = cls
