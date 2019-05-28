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
    __metaclass__ = ABCMeta

    def __init__(self, name, ref_frame_id="", offset_xy=(0, 0), offset_rot=0):
        self.name = name
        self.id = Manager.id
        self.tfs = [None] * 50
        self.par = None
        self._pos_offset = offset_xy
        self._rot_offset = offset_rot
        self.ref_frame_id = ref_frame_id
        Manager.id += 1

    @abstractmethod
    def _generate(self):
        pass

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
            self.tfs.append(tf)

        return True


class PatternFactory:
    __metaclass__ = ABCMeta

    _pattern_typs = {}

    @classmethod
    def reg_pattern_typ(cls, pattern_typ, pattern):
        cls._pattern_typs[pattern_typ] = pattern

    @classmethod
    def mk_pattern(cls, pattern_typ, base_params, pattern_params):
        pattern = cls._pattern_typs.get(pattern_typ)

        if not pattern:
            raise ValueError(pattern_typ)

        return pattern(base_params, **pattern_params)
