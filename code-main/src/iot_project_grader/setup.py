from setuptools import setup

package_name = 'iot_project_grader'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fede3751',
    maintainer_email='trombetti@di.uniroma1.it',
    description='Iot Project Grader. Displays the score of the project in a convenient window using the DearPyGui libraries',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'display=iot_project_grader.grade_display:main'
        ],
    },
)
