from setuptools import find_packages, setup

package_name = 'joy_communication'

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
    maintainer='tunotuno',
    maintainer_email='tunodatunoda0226@gmail.com',
    description='Joystick communication between PCs',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'joy_publisher = joy_communication.joy_publisher:main',
        'joy_subscriber = joy_communication.joy_subscriber:main',
        ],
    },
)
