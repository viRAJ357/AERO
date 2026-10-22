from setuptools import find_packages, setup
package_name = 'aero_perception'
setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nikhil',
    maintainer_email='nikhil@aero.local',
    description='AERO perception: YOLO, localization, semantic map',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detector = aero_perception.yolo_detector:main',
            'object_localizer = aero_perception.object_localizer:main',
            'semantic_map = aero_perception.semantic_map:main',
        ],
    },
)
