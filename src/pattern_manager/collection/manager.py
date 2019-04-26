from bidict import bidict


class Manager(object):
    def __init__(self, elements=[]):
        self.id = 0        
        self.active_element = 0
        self.iterator = 0
        self.has_finished = False
        self.elements = {}
        
        for e in elements:
            self.add_element(e)

    def add_element(self, element):
        self.elements[self.id] = element
        self.id += 1

    def remove_element(self, id):
        try:
            del self.elements[id]
            return True
        except KeyError:
            return False

    def pop_element(self, id):
        try:
            self.elements.pop(id)
        except KeyError:
            return False
    
    def set_active_element(self, id):
        if id in self.elements.keys():
            self.active_element = id
        else:
            return False
        
        return True
   
    def get_element(self, id):
        try:
            e = self.elements[id]
            return e
        except KeyError:
            return False

    def get_current_element(self):
        try:
            (i, e) = self.elements[self.active_element]
            return (i, e)
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
