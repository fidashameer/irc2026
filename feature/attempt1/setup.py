from setuptools import setup, find_packages

package_name = 'my_rob_con'

setup(
    name=package_name,
    version='0.0.0',
    package_dir={'': 'src'},  # Tell setuptools source lives in src/
    packages=find_packages(where='src', include=[package_name, f'{package_name}.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', ['src/my_rob_con/launch/nodes_launch.py']),
        (f'share/{package_name}/rado', ['src/my_rob_con/rado/best.pt']),
    ],
    install_requires=['setuptools', 'rclpy', 'opencv-python', 'ultralytics', 'cv_bridge'],
    zip_safe=True,
    maintainer='fidashameer',
    maintainer_email='fidashameer06@gmail.com',
    description='Dual robot perception and control package with cone detection and alignment',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cone_detection = my_rob_con.rado.cone_detection:main',
            'rotation_node = my_rob_con.rado.rotation_node:main',
            'movement_node = my_rob_con.rado.movement_node:main',
        ],
    },
)
