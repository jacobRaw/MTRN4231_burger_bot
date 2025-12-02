from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob('../launch/perception.launch.py')),
        
        (os.path.join('share', package_name), 
            [os.path.join('perception', 'black_seed.pt')]),

        (os.path.join('share', package_name), 
            [os.path.join('perception', 'burger_model.pt')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jacob',
    maintainer_email='jacobrawung1@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_vision = perception.yolo_vision:main',
        ],
    },
)
