import os

from glob import glob
from setuptools import find_packages, setup

package_name = 'trsa_lab1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'video'), glob('video/*')),    
        (os.path.join('share', package_name, 'calibration'), glob('calibration/*.yaml')),
        (os.path.join('share', package_name, 'calibration/images'), glob('calibration/images/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tmega',
    maintainer_email='tmega@todo.todo',
    description='TRSA Lab 1 package with camera drivers and calibration tools',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'image_reader               = trsa_lab1.image_reader:main',
            'camera_driver              = trsa_lab1.camera_driver:main',
            'image_converter            = trsa_lab1.image_converter:main',
            'camera_rectifier           = trsa_lab1.camera_rectifier:main',
            'object_detection           = trsa_lab1.object_detection:main',
            'camera_calibration_pub     = trsa_lab1.camera_calibration_pub:main',
        ],
    },
)
