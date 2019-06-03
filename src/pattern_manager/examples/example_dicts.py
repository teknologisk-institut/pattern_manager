#!/usr/bin/env python

linear_d = {
        'pattern_type': 'linear',
        'pattern_params': {
            'step_size': 0.1,
            'num_points': 3
        },
        'base_params': {
            'name': 'cheese_linear',
            'ref_frame_id': 'base_link',
            'offset_xy': [0.5, 0.3],
            'offset_rot': 0.2,
        }
    }

rect_d = {
    'pattern_type': 'rectangular',
    'pattern_params': {
        'num_points': (3, 2),
        'step_sizes': (0.1, 0.1),
    },
    'base_params': {
        'name': 'cheese_rect',
        'ref_frame_id': 'base_link',
    }
}

scatter_d = {
    'pattern_type': 'scatter',
    'pattern_params': {
        'point_list':
            [
                [1.0, 1.0, 1.0],
                [1.1, 1.1, 1.0],
                [1.2, 1.2, 1.0],
                [1.0, 1.0, 0.5, 0.0, 0.0, 0.5],
                [1.1, 1.1, 0.5, 0.0, 0.0, 0.6],
                [1.2, 1.2, 0.5, 0.0, 0.0, 0.7],
                [1.0, 1.0, 0.0, 0.0, 0.0, -0.5],
                [1.1, 1.1, 0.0, 0.0, 0.0, -0.6],
                [1.2, 1.2, 0.0, 0.0, 0.0, -0.7]
            ]
    },
    'base_params': {
        'name': 'cheese_scatter',
        'ref_frame_id': 'base_link',
    }
}

circle_d = {
    'pattern_type': 'circular',
    'pattern_params': {
        'r': 0.5,
        'num_points': 7,
    },
    'base_params': {
        'name': 'cheese_circle',
        'ref_frame_id': 'base_link',
    }
}