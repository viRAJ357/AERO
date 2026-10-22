from setuptools import find_packages, setup
package_name = 'aero_eval'
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
    description='AERO evaluation and metrics logging',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'metrics_logger = aero_eval.metrics_logger:main',
            'run_trials = aero_eval.run_trials:main',
            'generate_report = aero_eval.report:main',
        ],
    },
)
