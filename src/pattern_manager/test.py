#!/usr/bin/env python

# Copyright 2019 Danish Technological Institute (DTI)

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Author: Mads Vainoe Baatrup

import sys
import json
import yaml

from pattern_manager.xform import XForm


if __name__ == '__main__':
    t = XForm(name='root', parent=None, ref_frame='world')
    t1 = XForm(t)
    t2 = XForm(t)
    t3 = XForm(t)

    t4 = XForm(t2)
    t5 = XForm(t2)

    t6 = XForm(t4)
    t7 = XForm(t4)

    t8 = XForm(t7)
    t9 = XForm(t7)

    js = json.dumps(XForm.to_dict(), sort_keys=True, indent=4)
    dict_ = json.loads(js)

    XForm.recursive_remove_node(id(t))

    XForm.from_dict(dict_)
    print json.dumps(XForm.to_dict(), sort_keys=True, indent=4)

    # t.active = True
    # t2.active = True
    # t5.active = True

    # print "Active nodes:"

    # for n in XForm.get_active_nodes():
    #     print n.name

    # print XForm.get_active_nodes()[0]
    # XForm.iterate()
    # print XForm.get_active_nodes()[0]

    # while XForm.get_current_node():
    #     print XForm.get_current_node().name
    #     XForm.iterate()
    #
    # for i in range(10):
    #     x = XForm(t9)
    #
    # for n in XForm.get_nodes():
    #     print n.name

    # print ''
    #
    # print 'Found tree with id %d: %s' % (id(t5), XForm.get_node(id(t5)))
    #
    # print ''
    #
    # for n in XForm.get_nodes():
    #     print n.name
    #
    # XForm.remove_node(id(t2))
    #
    # print ''
    #
    # for n in XForm.get_nodes():
    #     print n.name
