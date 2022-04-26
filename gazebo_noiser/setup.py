from setuptools import setup

package_name = 'gazebo_noiser'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rodion',
    maintainer_email='rodion_anisimov@mail.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'noise_odom = gazebo_noiser.noise_odom:main',
            'plotter = gazebo_noiser.plotter:main'
        ],
    },
)
