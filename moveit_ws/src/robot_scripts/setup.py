from setuptools import setup

package_name = 'robot_scripts'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your-name',
    maintainer_email='your-email@example.com',
    description='Script to move Isaac Sim robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_to_joint_position = robot_scripts.move_to_joint_position:main',
            'move_on_scissors = robot_scripts.move_on_scissors:main',
        ],
    },
)
