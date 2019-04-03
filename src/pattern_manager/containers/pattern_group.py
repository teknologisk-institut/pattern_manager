from collections import OrderedDict

class PatternGroup():    
    def __init__(self, pattern=None):
        self.patterns = {}
        self.pattern_iterator = 0
        self.iter_parallel = False
        self.active_pattern = ''

        if pattern is not None:
            self.add_pattern(pattern)

    def _handle_name_duplicates(self, pattern):
        i = 0
        for p in self.patterns.values():
            if p.pattern_name.partition("(")[0] == pattern.pattern_name:
                i = i + 1

        if i > 0:
            pattern.pattern_name += '(' + str(i) + ')'

    def add_pattern(self, pattern):
        self._handle_name_duplicates(pattern)

        i = 0 if len(self.patterns) == 0 else max(self.patterns.keys()) + 1
        self.patterns[i] = pattern

    def remove_pattern(self, pattern_index):
        del self.patterns[pattern_index]

    def get_pattern_by_name(self, pattern_name, exact_match=False):
        matches = []
        for p in self.patterns.values():
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
        return self.patterns

    @property
    def pattern_iterator(self):
        return self.pattern_iterator

    @pattern_iterator.setter
    def pattern_iterator(self, iterator):
        self.pattern_iterator = iterator

    @property
    def iter_parallel(self):
        return self.iter_parallel

    @iter_parallel.setter
    def iter_parallel(self, iter_parallel):
        self.iter_parallel = iter_parallel

    @property
    def pattern_count(self):
        return len(self.patterns.keys())

    def is_pattern_finished(self, pattern_index):
        return self.patterns[pattern_index].is_finished()

    def reset_pattern(self, pattern_index):
        return self.patterns[pattern_index].reset_pattern()

    def get_point_iterator(self, pattern_index):
        return self.patterns[pattern_index].get_iterator()

    def set_point_iterator(self, pattern_index, iterator):
        self.patterns[pattern_index].set_iterator(iterator)

    @property
    def is_reverse_iteration(self, pattern_index):
        return self.patterns[pattern_index].is_reverse_iteration()

    def set_reverse_iteration(self, pattern_index, reverse):
        return self.patterns[pattern_index].set_reverse_iteration(reverse)

    @property
    def pattern_size(self, pattern_index):
        return self.patterns[pattern_index].get_pattern_size()

    @property
    def pattern_frame_id(self, pattern_index):
        return self.patterns[pattern_index].get_pattern_frame_id()

    def set_pattern_name(self, pattern_index, new_name):
        return self.patterns[pattern_index].set_pattern_name(new_name)

    @property
    def get_pattern(self, pattern_index):
        return self.patterns[pattern_index]

    def get_current_tf(self, pattern_name):
        (i, p) = self.get_current_pattern()
        return p.get_current_tf()

    def set_active_pattern(self, pattern_index):
        if pattern_index in self.patterns.keys():
            self.active_pattern = pattern_index
            return True
        else:
            return False

    def sorted_pattern_ids(self):
        return sorted(self.patterns.keys())

    def get_current_pattern(self):
        return self.active_pattern, self.patterns[self.active_pattern]

    def get_next_tf(self):
        (i, p) = self.get_current_pattern()
        next_i = p.get_iterator() + 1
        if not next_i < p.get_pattern_size():
            i = self.patterns.keys().index(i) + 1
            p = self.patterns[i]

            return p.get_tf_from_iter(0)
        else:
            return p.get_next_tf()

    def get_tf_from_iter(self, pattern_index, iter):
        return self.patterns[pattern_index].get_tf_from_iter(iter)

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
                self.active_pattern = k
                return k, self.patterns[k]
                
        return (False, False)