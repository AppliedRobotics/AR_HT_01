from setuptools import setup
import os
from glob import glob
package_name = 'AR_HT_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nuc',
    maintainer_email='rodion_anisimov@mail.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dif_control = AR_HT_bringup.dif_control:main',
            'MotorControl_odrive = AR_HT_bringup.MotorControl_odrive:main',
            'serial_connection = AR_HT_bringup.serial_script:main',
            'odom = AR_HT_bringup.wheel_odom:main',
            'ekf = AR_HT_bringup.ekf:main',
            'fixer = AR_HT_bringup.scan_fixer:main',
        ],
    },
)
