from setuptools import setup

PACKAGE_NAME = 'pipet_hand_mark7_teleop'

setup(
    name=PACKAGE_NAME,
    version='0.1.0',
    packages=[PACKAGE_NAME],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'teleop_keyboard = pipet_hand_mark7_teleop.teleop_keyboard:main',
            'grip_preset_node = pipet_hand_mark7_teleop.grip_preset_node:main',
        ],
    },
)
