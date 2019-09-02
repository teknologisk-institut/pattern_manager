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

from ..utils import tf_to_matrix, matrix_to_tf
from tf import transformations as tfs
from abc import ABCMeta
from ..manager import Manager

import numpy as np
import pluginlib


@pluginlib.Parent('pattern', group='patterns')
class Pattern:

    _instances = {}

    def __init__(self, nm, ref_frame_id="", offset_xy=(0, 0), offset_rot=0):

        self.name = nm
        self.type = 'Pattern'
        self.children = []
        self._pos_offset = offset_xy
        self._rot_offset = offset_rot
        self.ref_frame_id = ref_frame_id

        Pattern._instances[self.name] = self

        Manager.register_id(id(self))

    @staticmethod
    def get_pattern_by_name(nm):
        try:
            pat = Pattern._instances[nm]

            return pat
        except KeyError:
            print "Error: There exists no pattern with the name: {}".format(nm)

            return

    def offset_pattern(self, pattern):
        if not self._pos_offset == (0, 0):
            for pos in pattern:
                pos.translation.x += self._pos_offset[0]
                pos.translation.y += self._pos_offset[1]

        if not self._rot_offset == 0:
            transf_mat = tfs.compose_matrix(
                angles=[0, 0, self._rot_offset],
                translate=[self._pos_offset[0], self._pos_offset[1], 0])

            for pos in pattern:
                mat = tf_to_matrix(pos)
                new_pos = matrix_to_tf(np.dot(mat, transf_mat))
                pos.rotation = new_pos.rotation
                pos.translation = new_pos.translation

        return pattern

    def finish_generation(self, pattern, ignore_offset=False):
        pattern = pattern.reshape(pattern.size)

        if not ignore_offset:
            pattern = self.offset_pattern(pattern)

        for tf in pattern:
            self.children.append(tf)

        return True


class PatternFactory:
    """This class acts as a factory for generating patterns from a dictionary.
    """
    __metaclass__ = ABCMeta

    pattern_typs = {}

    @staticmethod
    def reg_pattern_typ(pattern_typ, pattern):
        """This function can be used to register new pattern types.
        
        :param pattern_typ: A pattern type descriptor.
        :type pattern_typ: str
        :param pattern: An object deriving from Pattern.
        :type pattern: Pattern
        """

        PatternFactory.pattern_typs[pattern_typ] = pattern

    @staticmethod
    def mk_pattern(pattern_typ, base_params, pattern_params):
        """This function returns a pattern of the specified type.
        
        :param pattern_typ: The requested pattern type.
        :type pattern_typ: str
        :param base_params: Parameters for the Pattern base-class.
        :type base_params: dict
        :param pattern_params: Paramters for the specific Pattern type.
        :type pattern_params: dict
        :raises ValueError: If pattern type does not exist.
        :return: A pattern of the specified type.
        :rtype: Pattern
        """

        pattern = PatternFactory.pattern_typs.get(pattern_typ)

        if not pattern:
            raise ValueError(pattern_typ)

        return pattern(base_params, **pattern_params)
