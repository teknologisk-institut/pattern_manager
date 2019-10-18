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

from pattern_manager.xform import XForm


if __name__ == '__main__':
    t = XForm('root', None)
    t1 = XForm('t1', t)
    t2 = XForm('t2', t)
    t3 = XForm('t3', t)

    t4 = XForm('t4', t2)
    t5 = XForm('t5', t2)

    t6 = XForm('t6', t4)
    t7 = XForm('t7', t4)

    t8 = XForm('t8', t7)
    t9 = XForm('t9', t7)

    t.active = True
    t2.active = True
    t5.active = True

    # print "Active nodes:"

    # for n in XForm.get_active_nodes():
    #     print n.name

    # print XForm.get_active_nodes()[0]
    # XForm.iterate()
    # print XForm.get_active_nodes()[0]

    while XForm.get_current_node():
        print XForm.get_current_node().name
        XForm.iterate()

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
