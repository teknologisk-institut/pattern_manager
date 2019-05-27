# #!/usr/bin/env python

# # Copyright 2019 Danish Technological Institute (DTI)

# # Licensed under the Apache License, Version 2.0 (the "License");
# # you may not use this file except in compliance with the License.
# # You may obtain a copy of the License at

# #     http://www.apache.org/licenses/LICENSE-2.0

# # Unless required by applicable law or agreed to in writing, software
# # distributed under the License is distributed on an "AS IS" BASIS,
# # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# # See the License for the specific language governing permissions and
# # limitations under the License.

# # Author: Mads Vainoe Baatrup

# from pattern_manager.collection import Manager
# from enum import Enum

# class GType(Enum):
#     group = 1
#     pattern = 2


# class Group(object):
# 	id = 0

# 	def __init__(self, typ, nm):
# 		self.typ = typ
# 		self.nm = nm
# 		self.id = Group.id
# 		self.grps = []
# 		self.pats = []
# 		self.par = None
# 		Group.id += 1

#     def add_subgroup(self, grp):
#         if not self.typ is GType.group:
#             print "Error: cannot add subgroup to group of type, %s", self.typ.name
#             return False
        
#         self.grps.append(grp)
#         grp.par = self

#     def add_pattern(pat):
#         if not self.typ is GType.pattern:
#             print "Error: cannot add pattern to group of type, %s" self.typ.name