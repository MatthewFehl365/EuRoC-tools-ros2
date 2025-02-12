from setuptools import setup

package_name = 'euroc_tools_ros_2'

setup(
    name=package_name,
    version='0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Matthew Fehl',
    author_email='matthew_fehl@live.com',
    maintainer='Matthew Fehl',
    maintainer_email='matthew_fehl@live.com',
    license='Apache License, Version 2.0',
    entry_points={
        'console_scripts': [
            'calibration_publisher = euroc_tools_ros_2.calibration_publisher:main',
        ],
    },
)