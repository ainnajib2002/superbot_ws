from setuptools import setup, find_packages

package_name = 'yolo_main'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    package_dir={'': '.'},  # <--- INI penting agar 'src/yolo_main' dikenali
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ban2aru',
    maintainer_email='qorskfo1023@hanmail.net',
    description='yolo for ros2 foxy version',
    license='AGPL-3.0 license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'yolo_node = yolo_main.yolo_node:main',
                'pose_estimation = yolo_main.pose_estimation:main',
                'yolo_follow = yolo_main.yolo_follow:main'
        ],
    },
)
