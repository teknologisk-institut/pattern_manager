#!/usr/bin/env python

linear_d = {
        'pattern_params': {
            'pattern_type': 'linear',
            'step_size': 0.1,
            'num_points': 3
        },
        'base_params': {
            'i': 0,
            'name': 'cheese_linear',
            'rev': False,
            'frame': 'base_link',
            'offset_xy': [0.5, 0.3],
            'offset_rot': 0.2,
        }
    }

linear_d2 = {
    'pattern_params': {
        'pattern_type': 'linear',
        'step_size': 0.1,
        'num_points': 3
    },
    'base_params': {
        'i': 0,
        'name': 'cheese_linear2',
        'rev': False,
        'frame': 'base_link',
        'offset_xy': [0.6, 0.4],
        'offset_rot': 0.2,
    }
}

rect_d = {
    'pattern_params': {
        'pattern_type': 'rectangular',
        'num_points': (3, 2),
        'step_sizes': (0.1, 0.1),
    },
    'base_params': {
        'i': 0,
        'name': 'cheese_rect',
        'rev': False,
        'frame': 'base_link',
    }
}

scatter_d = {
    'pattern_params': {
        'pattern_type': 'scatter',
        'point_list':
            [
                [1.0, 1.0, 1.0],
                [1.1, 1.0, 1.0],
                [1.2, 1.0, 1.0],
                [1.0, 1.0, 0.5, 0.0, 0.0, 0.5],
                [1.1, 1.0, 0.5, 0.0, 0.0, 0.6],
                [1.2, 1.0, 0.5, 0.0, 0.0, 0.7],
                [1.0, 1.0, 0.0, 0.0, 0.0, -0.5],
                [1.1, 1.0, 0.0, 0.0, 0.0, -0.6],
                [1.2, 1.0, 0.0, 0.0, 0.0, -0.7]
            ]
    },
    'base_params': {
        'i': 0,
        'name': 'cheese_scatter',
        'rev': False,
        'frame': 'base_link',
    }
}

circle_d = {

    'pattern_params': {
        'pattern_type': 'circular',
        'r': 0.5,
        'num_points': 7,
    },
    'base_params': {
        'i': 0,
        'name': 'cheese_circle',
        'rev': False,
        'frame': 'base_link',
    }
}