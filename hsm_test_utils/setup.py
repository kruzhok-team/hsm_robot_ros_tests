from setuptools import find_packages, setup

package_name = 'hsm_test_utils'

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
    description='The test utilities of the HSM-to-ROS2 framework',
    license='GPL-3.0-or-later',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
