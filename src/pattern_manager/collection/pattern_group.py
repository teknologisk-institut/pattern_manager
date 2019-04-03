from bidict import bidict


class PatternGroup(object):
    def __init__(self):
        self.elements = {}
        self.names = bidict()
        self.active_element = 0
        self.iterator = 0
        self.has_finished = False

    def _handle_duplicate_names(self, name):
        i = 0
        for n in self.names.values():
            if n.partition("(")[0] == name:
                i += 1

        return name if i == 0 else name + '(' + str(i) + ')'

    def add_element(self, element, id=0, name=''):
        self.elements[id] = element
        self.names[id] = self._handle_duplicate_names(name)

    def remove_element(self, id):
        try:
            del self.elements[id]
            return True
        except KeyError:
            return False

    def get_element_by_name(self, name):
        if name in self.names.keys():
            return self.elements[self.names.inverse[name]]
        else:
            return False
    
    def set_active_element(self, id):
        if id in self.elements.keys():
            self.active_element = id
        else:
            return False
        
        return True
    
    def get_current_element(self):
        try:
            (i, e) = self.elements[self.active_element]
            return (i, e)
        except KeyError:
            return False

    def get_element(self, id):
        try:
            e = self.elements[id]
            return e
        except KeyError:
            return False

    def increase_iterator(self):
        next_i = self.iterator + 1
        if next_i < len(self.elements):
            self.iterator += 1
        else:
            return False
        
        return next_i

    def element_count(self):
        return len(self.elements)

    def has_element_finished(self, id):
        try:
            self.elements[id].has_finished()
        except KeyError:
            return False

    def get_first_unfinished_element(self):
        keys = self.sorted_ids()
        for k in keys:
            if self.has_element_finished(k):
                continue
            else:
                self.active_element = k
                return k, self.elements[k]
                
        return False

    def sorted_ids(self):
        return sorted(self.elements.keys())

    def get_element_id(self, name):
        if name in self.names.values():
            return self.names.inverse[name]
        else:
            return False