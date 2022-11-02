from setuptools import setup
import os
from glob import glob
package_name = 'AR_HT_api'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'map'), glob('map/*.yaml')),
        (os.path.join('share', package_name, 'map'), glob('map/*.pgm')),
        (os.path.join('share', package_name, 'behaivor_trees'), glob('behaivor_trees/*.xml')),
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
        'api = AR_HT_api.ar_api:main',
        'manip_api = AR_HT_api.manip_api:main',
        'get_visual = AR_HT_api.visualization:main'
        ],
    },
)
