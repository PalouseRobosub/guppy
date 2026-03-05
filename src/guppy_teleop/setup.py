import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'guppy_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    include_package_data=True,
    package_data={
        'guppy_teleop.frontend' : ['assets.qrc', 'pyproject.toml'],
        'guppy_teleop.frontend.ui' : ['Main.qml', 'qmldir'],
        'guppy_teleop.frontend.ui.canvas' : ['*.qml'],
        'guppy_teleop.frontend.ui.sidebar' : ['*.qml'],
        'guppy_teleop.frontend.ui.widgets' : ['*.qml'],
        'guppy_teleop.frontend.ui.widgets.parameter' : ['*.qml'],
        'guppy_teleop.frontend.ui.toastify' : ['*.qml'],
        'guppy_teleop.frontend.workspaces' : ['*.json'],
        'guppy_teleop.frontend.icons' : ['*.svg'],
        'guppy_teleop.frontend.icons.toastify' : ['*.svg'],
        'guppy_teleop.frontend.workspaces' : ['*.json']
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.[pxy][yma]*'))),
    ],
    install_requires=['setuptools', 'pygame', 'PySide6'],
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
            'raw_controller = guppy_teleop.raw_controller:main',
            'translator = guppy_teleop.translator:main',
            'keyboard = guppy_teleop.keyboard:main',
            'terminal_backend = guppy_teleop.backend.terminal_backend:main',
            'terminal_frontend = guppy_teleop.frontend.terminal_frontend:main'
        ],
    },
)
