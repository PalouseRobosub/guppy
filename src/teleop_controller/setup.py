from setuptools import find_packages, setup

package_name = 'teleop_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pygame'],
    zip_safe=True,
    maintainer='robosub',
    maintainer_email='robosub@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'raw_controller = teleop_controller.raw_controller:main',
            'translator = teleop_controller.translator:main',
        ],
    },
)
