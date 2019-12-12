from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    package_dir={'': 'src'},
    packages=[
        'pattern_manager',
        'pattern_manager.plugins',
    ],
)

setup(**setup_args)
