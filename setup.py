## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['franka_lock_unlock'],
    # not used because it's not supported in catkin devel workspaces and installs scripts to global bin/
    # instead trivial entry scripts are provided in scripts/
    # entry_points={
    #     'console_scripts': [
    #         'franka_lock_unlock = franka_lock_unlock.franka_lock_unlock:main',
    #         'franka_shutdown = franka_lock_unlock.franka_shutdown:main',
    #     ],
    # },
)

setup(**setup_args)
