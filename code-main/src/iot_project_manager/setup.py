import os
from glob import glob

from setuptools import setup

package_name = 'iot_project_manager'


setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('lib', package_name), glob(package_name + '/math_utils.py*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fede3751',
    maintainer_email='trombetti@di.uniroma1.it',
    description='Iot Project Tester. This package is responsable of starting the Gazebo simulation where your solution will run',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'target_handler=iot_project_manager.target_handler:main'
        ],
    },
)
