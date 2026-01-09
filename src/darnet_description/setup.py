from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'darnet_description'

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
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'Bangkit = darnet_description.Bangkit:main',
            'Centerized = darnet_description.Centerized:main',
            'Jalan = darnet_description.Jalan:main',
            'PinnochioIK = darnet_description.PinnochioIK:main',
            'VisualTargetInteractive = darnet_description.InteractiveMarkerForCordinates:main',
            'SineWaveTarget = darnet_description.SineWaveTarget:main',
        ],
    },
)
