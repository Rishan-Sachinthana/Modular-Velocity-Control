from setuptools import setup

package_name = 'modular_velocity_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/formation.yaml']),
        ('share/' + package_name + '/launch', ['launch/modular_launch.py']),
        ('share/' + package_name + '/launch', ['launch/sim_launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/modular_mecanum_bot.urdf.xacro']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rishan',
    maintainer_email='sachinthana1996@gmail.com',
    description='Modular velocity dispatcher and visualizer for mecanum robots in ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'velocity_dispatcher = modular_velocity_control.velocity_dispatcher:main',
            'visualizer = modular_velocity_control.visualizer:main',
        ],
    },
)
