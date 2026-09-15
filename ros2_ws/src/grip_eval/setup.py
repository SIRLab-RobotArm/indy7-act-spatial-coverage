from setuptools import setup

package_name = 'grip_eval'

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
    description='Safe policy execution, teach-in, and rollout logging.',
    license='MIT',
    entry_points={'console_scripts': [
        'teach_target_node = grip_eval.teach_target_node:main',
        'policy_executor_node = grip_eval.policy_executor_node:main',
        'rollout_logger_node = grip_eval.rollout_logger_node:main',
        'replay_node = grip_eval.replay_node:main',
    ]},
)
