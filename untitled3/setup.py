from setuptools import find_packages, setup
import os
import glob

package_name = 'untitled3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch' , glob.glob(os.path.join('launch','*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jchj',
    maintainer_email='jchj@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'Robot_Server = untitled3.Robot_Server:main',
            'Robot_Control = untitled3.Robot_Control:main',
            'Cup_detect = untitled3.Cup_detect:main',
            'Guest_detect = untitled3.Guest_detect:main',
            'UI = untitled3.UI:main',
            'Voice_Input = untitled3.Voice_Input:main',
            'Voice_out = untitled3.Voice_out:main',
            'DB_manager = untitled3.DB_manager:main',
            'Hand_Interrupt = untitled3.Hand_Interrupt:main',
            'Hand_manual = untitled3.Hand_manual:main',
            'UpCam = untitled3.UpCam:main',
            'FrontCam = untitled3.FrontCam:main',
            'Container_Sealing_detect = untitled3.Container_Sealing_detect:main',
            'Guest_registration = untitled3.Guest_registration:main',
            'Human_detect = untitled3.Human_detect:main',
        ],
    },
)
