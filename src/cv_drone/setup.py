from setuptools import find_packages, setup

package_name = 'cv_drone'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch_sim.py']),
        ('share/' + package_name + '/launch', ['launch/launch_complex.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'teleop_node = cv_drone.teleop_node:main',
            'vision_node = cv_drone.vision_node:main',
            'pose_reporter_node = cv_drone.pose_reporter_node:main',
            'plotter_node = cv_drone.plotter_node:main'
        ],
    },
)
