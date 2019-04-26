from bidict import bidict


class Manager(object):
    def __init__(self, elements=[]):
        self.element_id = 0        
        self.iterator = 0
        self.finished = False
        self.elements = {}
        
        for e in elements:
            self.add_element(e)

    def add_element(self, element):
        self.elements[self.element_id] = element
        self.element_id += 1

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
   
    def get_element(self, id):
        try:
            e = self.elements[id]
            return e
        except KeyError:
            return False

    def get_current_element(self):
        try:
            (i, e) = self.iterator, self.elements[self.iterator]
            return (i, e)
        except KeyError:
            return False

    def get_next_element(self):
        next_i = self.iterator + 1
        if next_i < self.element_count():
            return next_i
        else:
            return False

    def increase_iterator(self):
        next_i = self.iterator + 1
        if next_i < self.element_count():
            self.iterator += 1
        else:
            self.finished = True
            return False
        
        return next_i

    def element_count(self):
        return len(self.elements)

    def has_finished(self):
        return self.finished

    def has_element_finished(self, id):
        if id < self.iterator:
            return True
        else:
            return False

    def sorted_ids(self):
        return sorted(self.elements.keys())
