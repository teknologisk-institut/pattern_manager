class PatternLayer():
    def __init__(self, group=None, group_id=0):
        self.groups = {}
        self.group_iterator = 0
        self.active_group = ''

        if group is not None:
            self.add_group(group, group_id)

    def add_group(self, group, group_id):
        self.groups[group_id] = group

    def remove_group(self, group_id):
        try:
            del self.groups[group_id]
            return True
        except KeyError:
            print 'error: group does not exist'
            return False

    def set_active_group(self, group_id):
        if group_id in self.groups.keys():
            self.active_group = group_id
            return True
        else:
            print 'error: group does not exist'
            return False

    def get_current_group(self):
        try:
            (i, g) = self.active_group, self.groups[self.active_group]
            return (i, g)
        except KeyError:
            print 'error: there is no active group'
            return False

    def get_group(self, group_id):
        try:
            g = self.groups[group_id]
            return g
        except KeyError:
            print 'error: group does not exist'
            return False
