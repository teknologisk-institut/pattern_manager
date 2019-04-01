class PatternGroup():
    _patterns = {}
    _pattern_iterator = 0
    _pattern_iter_parallel = False
    _active_pattern = ''
    
    def __init__(self, pattern=None):
        if pattern is not None:
            self.add_pattern(pattern)

    def _handle_name_duplicates(self, pattern):
        i = 0
        for p in self._patterns.keys():
            if p == pattern.pattern_name:
                i = i + 1

        if i > 0:
            pattern.pattern_name += '(' + str(i) + ')'

    def add_pattern(self, pattern):
        self._handle_name_duplicates(pattern)
        self._patterns[pattern.pattern_name] = pattern

    def remove_pattern(self, pattern):
        del self._patterns[pattern.pattern_name]

    def get_pattern_by_name(self, pattern_name):
        try:
            return self._patterns[pattern_name]
        except KeyError:
            print 'error: pattern does not exist'
            return False

    @property
    def patterns(self):
        return self._patterns

    @property
    def pattern_iterator(self):
        return self._pattern_iterator

    @pattern_iterator.setter
    def pattern_iterator(self, iterator):
        self._pattern_iterator = iterator

    @property
    def pattern_iter_parallel(self):
        return self._pattern_iter_parallel

    @pattern_iter_parallel.setter
    def pattern_iter_parallel(self, iter_parallel):
        self._pattern_iter_parallel = iter_parallel

    @property
    def pattern_count(self):
        return len(self._patterns.keys())

    @property
    def is_pattern_finished(self, pattern_name):
        return self._patterns[pattern_name].is_finished()

    def reset_pattern(self, pattern_name):
        return self._patterns[pattern_name].reset_pattern()

    def get_point_iterator(self, pattern_name):
        return self._patterns[pattern_name].get_iterator()

    def set_point_iterator(self, pattern_name, iterator):
        self._patterns[pattern_name].set_iterator(iterator)

    @property
    def is_reverse_iteration(self, pattern_name):
        return self._patterns[pattern_name].is_reverse_iteration()

    def set_reverse_iteration(self, pattern_name, reverse):
        return self._patterns[pattern_name].set_reverse_iteration(reverse)

    @property
    def pattern_size(self, pattern_name):
        return self._patterns[pattern_name].get_pattern_size()

    @property
    def pattern_frame_id(self, pattern_name):
        return self._patterns[pattern_name].get_pattern_frame_id()

    def set_pattern_name(self, pattern_name, new_name):
        return self._patterns[pattern_name].set_pattern_name(new_name)

    @property
    def get_pattern(self, pattern_name):
        return self._patterns[pattern_name]

    def get_current_tf_in_pattern(self, pattern_name):
        return self._patterns[pattern_name].get_current_tf()

    def set_active_pattern(self, pattern_name):
        if pattern_name in self._patterns.keys():
            self._active_pattern = pattern_name
            return True
        else:
            return False

    def sorted_pattern_ids(self):
        return sorted(self._patterns.keys())

    def get_current_pattern(self):
        return self._patterns[self._active_pattern]

    # def update_pattern(self, pattern_index, poses, fit_input_to_pattern, overwrite_orientation):
    #         print "%s input poses" % len(poses)
    #         if fit_input_to_pattern:
    #             # only create the pattern fitter when we need it
    #             if pattern_index not in self._fitters:
    #                 self._fitters[pattern_index] = pattern_fitter.PatternFitter(self._patterns[pattern_index])
    #         self._fitters[pattern_index].update_pattern(poses, fit_input_to_pattern, overwrite_orientation)
    #         self._patterns[pattern_index] = self._fitters[pattern_index].get_updated_pattern()
    #         return True

    def get_current_tf(self):
        p = self.get_current_pattern()
        return p.get_current_tf()

    def get_next_tf(self):
        (i, p) = self.get_current_pattern()
        next_pattern_i = p.get_iterator() + 1
        if not next_pattern_i < p.get_pattern_size():
            # using groups?
            if not self._patterns_are_grouped:
                return self.get_current_tf()
            # advance to next pattern
            next_index = self._patterns.keys().index(i) + 1
            p = self._patterns[next_index]
            return p.get_tf_from_iter(0)
        else:
            return p.get_next_tf()

    def get_tf_from_iter(self, pattern, iter):
        return self._patterns[pattern.pattern_name].get_tf_from_iter(iter)

    def increase_iterator(self):
        p = self.get_current_pattern()
        
        pattern_i = p.increase_iterator()
        if pattern_i is False:
            p = self.get_current_pattern()
            if i is False:
                return i
            else:
                pattern_i = 0

        return pattern_i