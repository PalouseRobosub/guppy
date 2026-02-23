from glob import glob
from setuptools import find_packages, setup

package_name = 'guppy_tasks'

setup(
    name=package_name,
    version='0.0.0',
    packages=['guppy_tasks', 'guppy_tasks.src'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.xml'),),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robosub',
    maintainer_email='colewilson.main@gmail.com',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
             'fcs_camera_listener = guppy_tasks.src.fcs_camera_listener:main',
             'fcs_target_tracker = guppy_tasks.src.fcs_target_tracker:main',
             'fcs_rangefinder = guppy_tasks.src.fcs_rangefinder:main',
             'fcs_targeting = guppy_tasks.src.fcs_targeting:main'
        ],
    },
)
