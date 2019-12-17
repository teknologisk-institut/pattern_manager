#!/usr/bin/env python

from pattern_manager import plugin

if __name__ == '__main__':
    loader = plugin.PluginLoader('plugins', 'pattern_manager')
    pat = loader.plugins['pattern_linear'](None, 'linear')
    print
