#!/usr/bin/env python3

import geometry_msgs.msg as gm

import numpy as np
import math

# TODO: How to include this in ROS2?
#import tf.transformations as tft 


# logging output
import logging
logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.DEBUG)


class Pattern(object):
    """ Base class for all patterns, with common interface"""

    _iterator = 0
    _parameterized = False
    _generated = False
    _finished = False

    def __init__(self, i=0, rev=False, frame="", name="", offset_xy=(0,0), offset_rot=0, order=[], static=False):
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

    # ITERATOR FUNCTIONS
    @property
    def iterator(self):
        return self._iterator

    @iterator.setter
    def iterator(self, iterator):
        self._iterator = iterator

    def increase_iterator(self):
        self.iterator += 1
        if not self.iterator < self._pattern.size:
            self._finished = True
            self.iterator -= 1
            return False
        return self.iterator

    def reset_iterator(self):
        self.iterator = 0

    def set_iteration_order(self, order):
        self._iteration_order = order
        return self.cleanup_iteration_order()

    def cleanup_iteration_order(self):
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
        # else == pattern size
        # reorder the pattern
        iter_order_np = np.array(self._iteration_order)
        self._pattern = self._pattern[iter_order_np]
        return True

    def is_finished(self):
        return self._finished

    def reset_pattern(self):
        self.reset_iterator()
        self._finished = False

    def get_pattern_size(self):
        return self._pattern.size

    def get_pattern_shape(self):
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
        if not self.parameterized:
            logging.error("Pattern is not parameterized, can't generate")
            self.generated = False
            return False
        return True

    # FRAME FUNCTIONS

    def set_all_frame_parameters(self, frame_id='', pattern_transform=gm.TransformStamped()):
        self.set_pattern_frame_id(frame_id)
        self.set_pattern_transform(pattern_transform)

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

    @property
    def pattern_name(self):
        return self._pattern_name

    @pattern_name.setter
    def pattern_name(self, name):
        self._pattern_name = name