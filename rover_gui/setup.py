from setuptools import setup
import os
from glob import glob

package_name = 'rover_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='azwadwakif',
    maintainer_email='wakifrajin@gmail.com',
    description='Mars Rover Control GUI',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rover_gui = rover_gui.combined_control_gui:main',
        ],
    },
)