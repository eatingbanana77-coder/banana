from setuptools import find_packages, setup

package_name = 'localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/ekf_local.yaml']),
        ('share/' + package_name + '/launch', ['launch/local_localization.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jinju',
    maintainer_email='wnwlswn23@gmail.com',
    description='ERP42 wheel odometry + EKF sensor fusion (robot_localization)',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'wheel_odometry = localization.wheel_odometry:main',
            'imu_time_sync = localization.imu_time_sync:main',
        ],
    },
)
