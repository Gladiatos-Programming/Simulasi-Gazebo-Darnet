from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'darnet_controller_test'

setup(
    name=package_name,                 # ← wajib underscore
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'),   glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dar',
    maintainer_email='dar@todo.todo',
    description='Controller test',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'joint_publisher = darnet_controller_test.joint_publisher:main',
            'CalcMass = darnet_controller_test.CalcMass:main',
            'Bangkit = darnet_controller_test.Bangkit:main',
            'Centerized = darnet_controller_test.Centerized:main',
        ],
    },
)
