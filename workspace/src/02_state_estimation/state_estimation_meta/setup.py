from setuptools import setup
import os
from glob import glob

package_name = 'state_estimation_meta'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        # Include all launch files. This is the most important line here!
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Wisconsin Autonomous',
    maintainer_email='wisconsinautonomous@studentorg.wisc.edu',
    description='Has metadata and launch files for state estimation packages',
    license='BSD3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
