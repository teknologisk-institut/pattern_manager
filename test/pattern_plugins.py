#!/usr/bin/env python

import pkg_resources

plugins = {
    entry_point.name: entry_point.load()
    for entry_point
    in pkg_resources.iter_entry_points('pattern_manager.plugins')
}

if __name__ == '__main__':
    linear = plugins['linear']
    linear.main()
