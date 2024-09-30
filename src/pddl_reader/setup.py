from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'pddl_reader'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name]),  # Automatically find packages
    data_files=[
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'data'), glob('data/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS 2 package for reading waypoints and using plansys2',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'pddl_reader = pddl_reader.pddl_reader:main',
        ],
    },
)
