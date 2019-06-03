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
from pattern_manager.utils import tf_to_matrix, matrix_to_tf, output, logging
from pattern_manager.collection import Manager
from tf import transformations as tfs
from abc import ABCMeta, abstractmethod
from pluginlib import PluginLoader

import numpy as np
import pluginlib


@pluginlib.Parent('pattern', group='patterns')
class Pattern:
    """This class is the base-class for all pattern plugins.
    """

    __metaclass__ = ABCMeta

    def __init__(self, name, ref_frame_id="", offset_xy=(0, 0), offset_rot=0):
        """The class constructor.
        
        :param name: The name to assign the pattern.
        :type name: str
        :param ref_frame_id: The id of the reference frame of the patterns transforms, defaults to ""
        :type ref_frame_id: str, optional
        :param offset_xy: The positional offset of a pattern, defaults to (0, 0)
        :type offset_xy: tuple, optional
        :param offset_rot: The rotational offset of a pattern, defaults to 0
        :type offset_rot: int, optional
        """

        self.nm = name
        self.typ = self.__class__.__bases__[0].name
        self.tfs = []
        self.par = None
        self._pos_offset = offset_xy
        self._rot_offset = offset_rot
        self.ref_frame_id = ref_frame_id
        
        Manager.register_id(id(self))

    @abstractmethod
    def _generate(self):
        """Generates a pattern with the parameters specified in the class constructor.
        """

        pass

    def offset_pattern(self, pattern):
        """This function allows a pattern to be positionally and rotationally offset.
        
        :param pattern: The pattern to be offset.
        :type pattern: Pattern
        :return: Returns the offset pattern.
        :rtype: Pattern
        """

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
        """This function flattens the initially generated tf list and adds them \
            to the class tf list. An optional tf offset ignore can also be set.
        
        :param pattern: The pattern to be added.
        :type pattern: Pattern
        :param ignore_offset: Whether to ignore tf offset or not, defaults to False
        :type ignore_offset: bool, optional
        :return: Returns True if generation succeeds
        :rtype: bool
        """
        pattern = pattern.reshape(pattern.size)

        if not ignore_offset:
            pattern = self.offset_pattern(pattern)

        for tf in pattern:
            self.tfs.append(tf)

        return True


class PatternFactory:
    """This class acts as a factory for generating patterns from a dictionary.
    """

    __metaclass__ = ABCMeta

    _pattern_typs = {}

    @staticmethod
    def reg_pattern_typ(pattern_typ, pattern):
        """This function can be used to register new pattern types.
        
        :param pattern_typ: A pattern type descriptor.
        :type pattern_typ: str
        :param pattern: An object deriving from Pattern.
        :type pattern: Pattern
        """

        PatternFactory._pattern_typs[pattern_typ] = pattern

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

        pattern = PatternFactory._pattern_typs.get(pattern_typ)

        if not pattern:
            raise ValueError(pattern_typ)

        return pattern(base_params, **pattern_params)
