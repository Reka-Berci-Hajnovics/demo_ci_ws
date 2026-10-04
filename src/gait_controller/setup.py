from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gait_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='reka',
    maintainer_email='reka.hajnovics@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gait_main_node = gait_controller.gait_main_node:main',
            'cyl_pos_pub_node = gait_controller.cylinder_pos_pub:main',
            'gait_sequencer_time_action_node = gait_controller.gait_sequencer_time_action_node:main',
            'pump_frequency_publisher_node = gait_controller.pump_frequency_publisher_node:main',
            'foxglove_button_action_bridge_node = gait_controller.foxglove_button_action_bridge_node:main',

        ],
    },
)
