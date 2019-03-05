from __future__ import division
from base_pattern import Pattern

import utilities
import numpy as np


class PatternLinear(Pattern):
    """ Points along x-axis of origin frame by default """
    _points = 0
    _step_size = 0
    _length = 0
    _axis = 'x'

    def set_pattern_parameters(self, number_of_points=0, step_size=0, line_length=0, axis='x'):
        try:
            (po, st, le) = utilities.handle_input_1d(number_of_points, step_size, line_length)
        except TypeError:
            return False
        self._step_size = st
        self._points = po
        self._length = le
        self._axis = axis
        self.set_parameterized(True)
        return True

    def generate_pattern(self):
        if not self.can_generate():
            return False
        self._pattern = utilities.frames_along_axis(self._points,
                                          self._step_size,
                                          axis=self._axis)
        self._pattern_org_copy = np.copy(self._pattern)
        self.finish_generation()
        self._pattern_org_copy = np.copy(self._pattern)
        return True


def main():
    print('I\'m a linear plugin')


if __name__ == '__main__':
    main()
