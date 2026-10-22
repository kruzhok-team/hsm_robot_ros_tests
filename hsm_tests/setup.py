from setuptools import find_packages, setup

package_name = 'hsm_tests'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alexey Fedoseev',
    maintainer_email='aleksey@fedoseev.net',
    description='The testing environment for the HSM-to-ROS2 framework',
    license='GPL-3.0-or-later',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fake_robot = hsm_tests.fake_robot.fake_robot:main',
            'hsm_robot_test = hsm_tests.diagram_test:main',
            'hsm_turtle_run = hsm_tests.turtle_run:main',
        ],
    },
)
