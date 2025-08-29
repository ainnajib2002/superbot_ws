from setuptools import setup
from glob import glob
import os

package_name = 'superbot_core'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],          # modul python (boleh kosong)
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # install semua launch file
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bonaubu20',
    maintainer_email='you@example.com',
    description='Robot supermarket GUI + navigation',
    license='MIT',
    # ------ ini kuncinya ------
    entry_points={
        'console_scripts': [
            #  nama ros2 run     = modul_python:func
            'superggui_yolo = superbot_core.superggui_yolo:main',
        ],
    },
    scripts=['scripts/superggui_yolo.py'],  # supaya file ikut ter-install & punya izin exec
)
