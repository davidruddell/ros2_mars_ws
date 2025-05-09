from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'mars_rover_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        # Install the package.xml
        ('share/' + package_name, ['package.xml']),

        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        # Install model files (SDFs, meshes, etc.)
        (os.path.join('share', package_name, 'models'), glob('models/*.sdf')),

        # Optionally include meshes, if you add them later
        # (os.path.join('share', package_name, 'models', 'meshes'), glob('models/meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David Ruddell',
    maintainer_email='davidwruddell@gmail.com',
    description='Mars rover control package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motion_command = mars_rover_control.motion_command:main',
            'obstacle_detector = mars_rover_control.obstacle_detector:main',
            #'spawn_entities = mars_rover_control.spawn_entities:main',
        ],
    },
)
