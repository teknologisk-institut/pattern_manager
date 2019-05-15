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


""" Provides a common base class for patterns, realized as plugins.

This file is not meant to be used directly."""

import geometry_msgs.msg as gm
import pluginlib
import numpy as np
from pattern_manager.utils import transform_to_matrix, matrix_to_transform, output, logging
from pattern_manager.collection import Manager
from tf import transformations as tfs


@pluginlib.Parent('pattern', group='patterns')
class Pattern(object):
    """Base class for all patterns, with common interface.

    Provides a common base class for all patterns, with common methods for iteration, generation, cleanup etc.

    :param i: start iterator at this index (optional)
    :type i: int
    :param rev: iterate backwards (from n, rather than 0) (optional)
    :type rev: bool
    :param frame: frame name that this pattern is relative to (optional)
    :type frame: str
    :param name: name of the pattern (optional)
    :type name: str
    :param offset_xy: x and y position offset wrt. the frame (optional)
    :type offset_xy: tuple
    :param offset_rot: z rotation (yaw) offset wrt. the frame (optional)
    :type offset_rot: float
    :param order: explicit iteration order (optional)
    :type order: list
    """

    def __init__(self, i=0, rev=False, frame="", name="", offset_xy=(0, 0), offset_rot=0, order=[]):
        self._manager = Manager("{}_manager".format(name))
        self.parent = None
        self.name = name
        self.pattern_frame_id = frame
        self._iteration_order = order
        self._pos_offset = tuple(offset_xy)
        self._rot_offset = offset_rot
        self.pattern_transform = gm.TransformStamped()
        self._pattern = np.array(np.empty(0), dtype=gm.Transform)

    @property
    def finished(self):
        return self._manager.finished

    @finished.setter
    def finished(self, f):
        self._manager.finished = f

    @property
    def iterator(self):
        return self._manager.iterator

    @iterator.setter
    def iterator(self, i):
        self._manager.iterator = i

    @property
    def active(self):
        return self._manager.active

    @active.setter
    def active(self, a):
        self._manager.active = a

    @property
    def elements(self):
        return self._manager.elements

    def iterate(self):
        self._manager.iterate()

    def get_pattern_size(self):
        return self._manager.element_count

    def add_element(self, e):
        self._manager.add_element(e)
    
    def remove_element(self, i):
        self._manager.remove_element(i)

    def pop_element(self, i):
        self._manager.pop_element(i)

    def get_element(self, i):
        return self._manager.get_element(i)

    def get_element_index(self, e):
        return self._manager.get_element_index(e)

    def get_current_element(self):
        return self._manager.get_current_element()

    def get_next_element(self):
        return self._manager.get_next_element()

    def reset(self):
        self._manager.reset()

    @property
    def pattern_frame_id(self):
        """Frame that the pattern positions are with reference to.

        :type: str
        """
        return self._pattern_frame_id

    @pattern_frame_id.setter
    def pattern_frame_id(self, frame_name):
        self._pattern_frame_id = frame_name

    @property
    def pattern_transform(self):
        """Transform of the pattern, wrt. pattern_frame_id.

        :getter: Return the transform
        :setter: Set the transform. Fills in frame_id with pattern_frame_id, if unspecified.
        :type: gm.Transform
        """
        return self._pattern_transform

    @pattern_transform.setter
    def pattern_transform(self, transform):
        self._pattern_transform = transform
        if transform.header.frame_id == "":
            self._pattern_transform.header.frame_id = self.pattern_frame_id

    def set_iteration_order(self, order):
        """Set iteration order explicitly.

        The order is specified as a list of indices. len(order) should be less than or equal to the pattern size.

        :param order: iteration order (list of indices)
        :type order: list
        :return: success
        :rtype: bool
        """
        self._iteration_order = order
        return self.cleanup_iteration_order()

    def cleanup_iteration_order(self):
        """Truncates or expands the current iteration order.

        This function makes sure the length of the internal iteration order is the same as the pattern size.
        If iteration order is shorter, it is expanded with missing indices. If it is longer, it is simply truncated.

        :return: success
        :rtype: bool
        """
        # make sure we have an order the exact length of the pattern
        if len(self._iteration_order) == 0:
            self._iteration_order = range(self.get_pattern_size())
        elif len(self._iteration_order) > self.get_pattern_size():
            output.warning("Specified iteration order is larger than pattern size, cropping order")
            self._iteration_order = self._iteration_order[:self.get_pattern_size()]
        elif len(self._iteration_order) < self.get_pattern_size():
            # find uniques
            orig_order = range(self.get_pattern_size())
            for o in self._iteration_order:
                orig_order.remove(o)
            self._iteration_order += orig_order
        # reorder the pattern
        iter_order_np = np.array(self._iteration_order)
        self._pattern = self._pattern[iter_order_np]
        return True

    def set_all_frame_parameters(self, frame_id, pattern_transform=gm.TransformStamped()):
        """Sets frame and transform from frame to pattern.

        :param frame_id: Name of the pattern frame in the tf tree
        :param frame_id: str
        :param pattern_transform: Transform from frame to pattern origin (1st position), defaults to zero transform
        :param pattern_transform: geometry_msgs.TransformStamped, optional
        """

        if not frame_id == "":
            self.pattern_frame_id = frame_id
        self.pattern_transform = pattern_transform

    def offset_pattern(self):
        """Correctly offsets each position in the pattern.

        Will modify each position in the pattern, typically after generation, with the pattern offset from the parent frame to xy and yaw offset specified during initialization.

        """

        # no offset
        if self._pos_offset == (0, 0) and self._rot_offset == 0:
            return
        # offset
        # just translate
        if not self._pos_offset == (0, 0) and self._rot_offset == 0:
            for pos in self._pattern:
                pos.translation.x += self._pos_offset[0]
                pos.translation.y += self._pos_offset[1]
            return
        # with rotation
        if not self._rot_offset == 0:
            transf_mat = tfs.compose_matrix(angles=[0, 0, self._rot_offset],
                                            translate=[self._pos_offset[0], self._pos_offset[1], 0])
            for pos in self._pattern:
                mat = transform_to_matrix(pos)
                new_pos = matrix_to_transform(np.dot(mat, transf_mat))
                pos.rotation = new_pos.rotation
                pos.translation = new_pos.translation
            return

    def finish_generation(self, ignore_offset=False):
        """Mark generation finished, and do various cleanup.

        Offsets the pattern, if specified, and cleans up the iteration order, if specified.

        :param ignore_offset: Will ignore the xy and yaw offset given during intialization, defaults to False
        :param ignore_offset: bool, optional
        :return: Whether or not the pattern is correctly generated.
        :rtype: bool
        """
        
        self._pattern = self._pattern.reshape(self._pattern.size)
        if not ignore_offset:
            self.offset_pattern()
        self.cleanup_iteration_order()
        # self.generated = True
        return True
