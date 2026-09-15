from setuptools import setup

package_name = 'grip_collect'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SirLab',
    maintainer_email='sirlab@todo.todo',
    description='Synchronized incremental HDF5 demonstration recorder.',
    license='MIT',
    entry_points={'console_scripts': [
        'episode_recorder_node = grip_collect.episode_recorder_node:main',
    ]},
)
