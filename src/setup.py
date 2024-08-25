from setuptools import find_packages, setup

package_name = 'camera_ar_marker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'opencv-python'],
    zip_safe=True,
    maintainer='messi',
    maintainer_email='messi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_publisher = camera_ar_marker.camera_publisher:main',
            'ar_marker_subscriber = camera_ar_marker.ar_marker_subscriber:main',
            'camera_fps = camera_ar_marker.camera_fps:main',
            'compressed_fps = camera_ar_marker.compressed_fps:main',
            'cheese = camera_ar_marker.cheese:main', 
            'depthpoint_subscriber = camera_ar_marker.depthpoint_subscriber:main',
            #'test_ar_marker_subscriber = camera_ar_marker.test_ar_marker_subscriber:main',
            'ar_marker_subscriber_executor = camera_ar_marker.ar_marker_subscriber_executor:main', 
            'rotate_robot = camera_ar_marker.rotate_robot:main',
            'cmd_vel_listener = camera_ar_marker.cmd_vel_listener:main',
            'rotate_robot_test = camera_ar_marker.rotate_robot_test:main',
            'turtlesim_rotate = camera_ar_marker.turtlesim_rotate:main',
            'ar_marker_test = camera_ar_marker.ar_marker_test:main',
            'cmd_vel_controller = camera_ar_marker.cmd_vel_controller:main',
        ],
    },
)
