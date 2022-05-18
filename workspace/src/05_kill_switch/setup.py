from setuptools import setup

package_name = 'kill_switch'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wagrandprix',
    maintainer_email='wagrandprix@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "kill_switch = kill_switch.kill_switch:main",
            "kill_switch_dummy = kill_switch.kill_switch_dummy:main",
        ],
    },
)
