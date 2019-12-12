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

from pattern_manager.pattern import Pattern
from pattern_manager.util import handle_input_1d
from pattern_manager.xform import XForm


class LinearPattern(Pattern):
    """
    This plugin class specifies the attributes of, and is responsible for the generation of, a linear XForm pattern

    :param parent: An XForm parent object under which to create the XForm pattern
    :type parent: XForm
    :param num_points: The number of points which make up the pattern
    :type num_points: int, optional
    :param step_size: The distance between each XForm's position in the pattern
    :type step_size: float, optional
    :param line_len: The total length of the pattern
    :type line_len: float, optional
    """

    # The plugin alias
    _alias_ = 'linear'

    def __init__(self, parent, num_points=0, step_size=0.0, line_len=0.0):
        super(LinearPattern, self).__init__(parent)

        self.num_points = num_points
        self.step_size = step_size
        self.line_len = line_len

    def generate(self):
        """
        This function generates the XForm pattern from the instance attributes
        """

        try:
            (po, st, le) = handle_input_1d(self.num_points, self.step_size, self.line_len)
        except TypeError, e:
            rospy.logerr(e)

            return None

        x_set = set()

        for i in range(po):
            x_set.add(i * st)

        c = 0
        for x in x_set:
            tf = XForm(self.parent, name='{}_{}'.format(self.parent.name, c))
            tf.translation.x = x

            c += 1
