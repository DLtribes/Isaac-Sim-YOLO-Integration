from setuptools import find_packages, setup
package_name = 'yolov8_detector'
setup(
    name=package_name,
    version='0.0.0',
    packages=['yolov8_detector'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'cv_bridge', 'ultralytics'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolov8_node = yolov8_detector.yolov8_node:main'
        ],
    },
)
