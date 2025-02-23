from setuptools import find_packages, setup

package_name = 'rover_4wheels'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bernd',
    maintainer_email='bernd@todo.todo',
    description='4Wheel Rover. Jedes Rad einzeln lenkbar',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rover_node=rover_4wheels.Rover:main',
        ],
    },
)
