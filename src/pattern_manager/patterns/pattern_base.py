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

from enum import Enum
from pluginlib import PluginLoader


class PType(Enum):
    lin = 1
    rect = 2
    circ = 3
    scat = 4

class Pattern(object):
    id = 0

    def __init__(self, typ, nm):
        self.typ = typ
        self.nm = nm
        self.id = Pattern.id
        self.tfs = [None] * 50
        self.step_size = 0
        self.len = 0
        self.no_pts = 0
        self.par = None
        Pattern.id += 1
        