from setuptools import setup
import os
from glob import glob

package_name = 'iot_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('lib', package_name), glob(package_name + '/sim_utils.py*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fede3751',
    maintainer_email='trombetti@di.uniroma1.it',
    description='IoT Project main package. Used only to store the launch file which will use all the packages together',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
