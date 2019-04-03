from collections import OrderedDict

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
        for p in self._patterns.values():
            if p.pattern_name.partition("(")[0] == pattern.pattern_name:
                i = i + 1

        if i > 0:
            pattern.pattern_name += '(' + str(i) + ')'

    def add_pattern(self, pattern):
        self._handle_name_duplicates(pattern)

        i = 0 if len(self._patterns) == 0 else max(self._patterns.keys()) + 1
        self._patterns[i] = pattern

    def remove_pattern(self, pattern_index):
        del self._patterns[pattern_index]

    def get_pattern_by_name(self, pattern_name, exact_match=False):
        matches = []
        for p in self._patterns.values():
            if exact_match:
                if p.pattern_name == pattern_name:
                    return p
            else:
                if pattern_name.lower() in p.pattern_name.lower():
                    matches.append(p)
        
        if len(matches) == 0:
            return False
        else:
            return matches

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

    def is_pattern_finished(self, pattern_index):
        return self._patterns[pattern_index].is_finished()

    def reset_pattern(self, pattern_index):
        return self._patterns[pattern_index].reset_pattern()

    def get_point_iterator(self, pattern_index):
        return self._patterns[pattern_index].get_iterator()

    def set_point_iterator(self, pattern_index, iterator):
        self._patterns[pattern_index].set_iterator(iterator)

    @property
    def is_reverse_iteration(self, pattern_index):
        return self._patterns[pattern_index].is_reverse_iteration()

    def set_reverse_iteration(self, pattern_index, reverse):
        return self._patterns[pattern_index].set_reverse_iteration(reverse)

    @property
    def pattern_size(self, pattern_index):
        return self._patterns[pattern_index].get_pattern_size()

    @property
    def pattern_frame_id(self, pattern_index):
        return self._patterns[pattern_index].get_pattern_frame_id()

    def set_pattern_name(self, pattern_index, new_name):
        return self._patterns[pattern_index].set_pattern_name(new_name)

    @property
    def get_pattern(self, pattern_index):
        return self._patterns[pattern_index]

    def get_current_tf(self, pattern_name):
        (i, p) = self.get_current_pattern()
        return p.get_current_tf()

    def set_active_pattern(self, pattern_index):
        if pattern_index in self._patterns.keys():
            self._active_pattern = pattern_index
            return True
        else:
            return False

    def sorted_pattern_ids(self):
        return sorted(self._patterns.keys())

    def get_current_pattern(self):
        return self._active_pattern, self._patterns[self._active_pattern]

    def get_next_tf(self):
        (i, p) = self.get_current_pattern()
        next_i = p.get_iterator() + 1
        if not next_i < p.get_pattern_size():
            i = self._patterns.keys().index(i) + 1
            p = self._patterns[i]

            return p.get_tf_from_iter(0)
        else:
            return p.get_next_tf()

    def get_tf_from_iter(self, pattern_index, iter):
        return self._patterns[pattern_index].get_tf_from_iter(iter)

    def increase_pattern_iterator(self):
        (i, p) = self.get_current_pattern()
        
        pattern_i = p.increase_iterator()
        if pattern_i is False:
            p = self.get_current_pattern()
            if i is False:
                return i
            else:
                pattern_i = 0

        return pattern_i

    def get_first_unfinished_pattern(self):
        keys = self.sorted_pattern_ids()
        for k in keys:
            if self.is_pattern_finished(k):
                continue
            else:
                self._active_pattern = k
                return k, self._patterns[k]
                
        return (False, False)