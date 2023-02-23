from setuptools import setup

package_name = 'gps_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Quinn Cypher',
    maintainer_email='tmqCypher.git@gmail.com',
    description='A ROS 2 node implementing the Qwiic_Ublox_Gps_Py library',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_controller = gps_controller.GPSController:main',
            'gps_benchmarker = gps_controller.GPSBenchmarker:main',
        ],
    },
)
