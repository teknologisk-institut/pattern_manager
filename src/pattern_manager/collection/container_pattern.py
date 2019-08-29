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

from container import Container
from manager import Manager


class ContainerPattern(Container):

    def __init__(self, nm, par=None):
        super(ContainerPattern, self).__init__(nm, par)

        Manager.register_id(id(self))

    # def get_pattern_by_name(self, nm):
    #     for p in self.children:
    #
    #         if not p.name == nm:
    #             continue
    #
    #         return p
