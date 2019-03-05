from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    name='pattern_manager',
    package_dir={'': 'src'},
    packages=[
        'base_pattern',
        'linear_pattern',
        'rectangular_pattern',
        'scatter_pattern',
        'utilities'
    ],
    entry_points={
        'pattern_manager.plugins': [
            'linear = linear_pattern.pattern',
            'rect = rectangular_pattern.pattern',
            'scatter = scatter_pattern.pattern',
        ]
    }
)

setup(**setup_args)
