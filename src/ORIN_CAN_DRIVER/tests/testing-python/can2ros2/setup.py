from setuptools import setup

package_name = 'can2ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mechatronics',
    maintainer_email='mechatronics@todo.todo',
    description='1st attempt at pushing CAN bus data to Ros2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'canPublisher = can2ros2.can2ros2:main',
        ],
    },
)
