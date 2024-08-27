from setuptools import find_packages, setup

package_name = 'omni_control_base'

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
    maintainer='Gabriel Rodriguez',
    maintainer_email='gabearod2@gmail.com',
    description='Omnidirectional Control for deployment on the Jetson Nano.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_control_node = omni_control_base.pid_control_node:main',
            'box_publisher_node = omni_control_base.box_publisher_node:main',
            'motor_cmd_sender_node = omni_control_base.motor_cmd_sender_node:main'
        ],
    },
)
