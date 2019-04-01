class PatternGroup():
    _patterns = {}
    _iterator = 0
    _iter_parallel = False
    
    def __init__(self, pattern=None):
        if pattern is not None:
            self.add_pattern(pattern)

    def _resolve_pattern_name(self, pattern):
        i = 0
        for p in self._patterns.keys():
            if p == pattern.pattern_name:
                i = i + 1

        if i > 0:
            pattern.pattern_name += '(' + str(i) + ')'

    def add_pattern(self, pattern):
        self._resolve_pattern_name(pattern)
        self._patterns[pattern.pattern_name] = pattern

    def remove_pattern(self, pattern):
        del self._patterns[pattern.pattern_name]

    @property
    def patterns(self):
        return self._patterns

    @property
    def iterator(self):
        return self._iterator

    @iterator.setter
    def iterator(self, iterator):
        self._iterator = iterator

    @property
    def iter_parallel(self):
        return self._iter_parallel

    @iter_parallel.setter
    def iter_parallel(self, iter_parallel):
        self._iter_parallel = iter_parallel