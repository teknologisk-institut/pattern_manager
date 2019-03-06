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

from .. import utils

import geometry_msgs.msg as gm
import numpy as np
import pluginlib
from pattern_manager.utils import *
from tf import transformations as tfs

# logging output
import logging
logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.DEBUG)


@pluginlib.Parent('pattern')
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
    :param static: lock the pattern for any future updates (optional)
    :type static: bool
    """

    _iterator = 0
    _parameterized = False
    _generated = False
    _finished = False

    def __init__(self, i=0, rev=False, frame="", name="", offset_xy=(0, 0), offset_rot=0, order=[], static=False):
        self.iterator = i
        self.reverse_iteration = rev
        self.pattern_frame_id = frame
        self.pattern_name = name
        self._iteration_order = order
        self._static = static
        self._pos_offset = tuple(offset_xy)
        self._rot_offset = offset_rot
        self.pattern_transform = gm.TransformStamped()
        self._pattern = np.array(np.empty(0), dtype=gm.Transform)

    @pluginlib.abstractmethod
    def generate_pattern(self):
        pass

    # ITERATOR FUNCTIONS
    @property
    def iterator(self):
        return self._iterator

    @iterator.setter
    def iterator(self, iterator):
        self._iterator = iterator

    def increase_iterator(self):
        """Increase the iterator by 1.
        
        :return: the new value of the iterator, or False if the pattern was finished
        :rtype: int if successful, bool if pattern was already finished 
        """
        self.iterator += 1
        if not self.iterator < self._pattern.size:
            self._finished = True
            self.iterator -= 1
            return False
        return self.iterator

    def reset_iterator(self):
        """Reset iterator to 0."""
        self.iterator = 0

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
            logging.warning("Specified iteration order is larger than pattern size, cropping order")
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

    def is_finished(self):
        return self._finished

    def reset_pattern(self):
        """Resets a pattern, by resetting iterator, and marking the pattern not finished."""
        self.reset_iterator()
        self._finished = False

    def get_pattern_size(self):
        """Returns the size of the pattern in 1 dimension (number of indices)."""
        return self._pattern.size

    def get_pattern_shape(self):
        """Returns the shape of the pattern.

        :return: shape of pattern
        :rtype: tuple
        """
        return self._pattern.shape

    # DIRECTION FUNCTIONS
    @property
    def reverse_iteration(self):
        return self._reverse_iteration

    @reverse_iteration.setter
    def reverse_iteration(self, reverse):
        self._reverse_iteration = reverse

    # INITIALIZED FUNCTIONS
    @property
    def generated(self):
        return self._generated

    @generated.setter
    def generated(self, g):
        self._generated = g

    @property
    def parameterized(self):
        return self._parameterized

    @parameterized.setter
    def parameterized(self, parameterized):
        self._parameterized = parameterized

    def can_generate(self):
        """Check if the pattern has been correctly paramterized.
        
        :return: Can generate
        :rtype: bool
        """

        if not self.parameterized:
            logging.error("Pattern is not parameterized, can't generate")
            self.generated = False
            return False
        return True

    # FRAME FUNCTIONS

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

    @property
    def pattern_frame_id(self):
        return self._pattern_frame_id
    
    @pattern_frame_id.setter
    def pattern_frame_id(self, frame_name):
        self._pattern_frame_id = frame_name

    @property
    def pattern_transform(self):
        return self._pattern_transform

    @pattern_transform.setter
    def pattern_transform(self, transform):
        self._pattern_transform = transform
        if transform.header.frame_id == "":
            self._pattern_transform.header.frame_id = self.pattern_frame_id

    @property
    def pattern_name(self):
        return self._pattern_name

    @pattern_name.setter
    def pattern_name(self, name):
        self._pattern_name = name

    # OFFSET FUNCTIONS

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
        self.generated = True
        return True

    def get_tf_from_iter(self, i):
        """Get the transformation corresponding to a specific index in the pattern.

        :param i: index of the position to query
        :type i: int
        :return: Transform from pattern parent frame to requested position index, if available
        :rtype: geometry_msgs.TransformStamped, False otherwise
        """
        try:
            if len(self._pattern) == 0:
                return False
            if self.reverse_iteration:
                index = -(i + 1)
            else:
                index = i
            t = self._pattern.item(index)
            return t
        except IndexError:
            logging.error("Iterator value %s exceeded dimension of pattern size %s" % (i, self._pattern.shape))
            return False

    def get_current_tf(self):
        """Get the transform corresponding to the current active pattern position.

        :return: Transform of current position.
        :rtype: geometry_msgs.TransformStamped
        """
        return self.get_tf_from_iter(self.iterator)

    def get_next_tf(self):
        """Get the transform corresponding to the next pattern position.

        :return: Transform of next position.
        :rtype: geometry_msgs.TransformStamped
        """
        i = self.iterator + 1
        return self.get_tf_from_iter(i)
