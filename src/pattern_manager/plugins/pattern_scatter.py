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

import rospy

from pattern_manager.plugin import Plugin
from pattern_manager.xform import XForm


class ScatterPattern(Plugin):
    """
    This plugin class specifies the attributes of, and is responsible for the generation of, a scattered XForm pattern

    :param parent: An XForm parent object under which to create the XForm pattern
    :type parent: XForm
    :param points: The points which make up the pattern
    :type points: list, optional
    """

    def __init__(self, parent, points):
        super(ScatterPattern, self).__init__(parent)

        self.points = points

    def generate(self):
        """
        This function generates the XForm pattern from the instance attributes
        """

        if not len(self.points) > 0:
            rospy.logerr("Scatter points cannot be zero")

            return None

        xyz_set = set()

        for p in self.points:
            xyz_set.add((p[0], p[1], p[2]))

        i = 0
        for xyz in xyz_set:
            tf = XForm(self.parent, name='{}_{}'.format(self.parent.name, i))

            tf.translation.x = xyz[0]
            tf.translation.y = xyz[1]
            tf.translation.z = xyz[2]

            i += 1
