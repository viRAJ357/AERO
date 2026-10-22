from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'aero_dashboard'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'static'), glob('static/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nikhil',
    maintainer_email='nikhil@aero.local',
    description='AERO Web Dashboard server',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dashboard_server = aero_dashboard.server:main',
        ],
    },
)
