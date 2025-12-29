from setuptools import setup

package_name = 'servo_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gerald Rocher',
    maintainer_email='gerald.rocher@univ-cotedazur.fr',
    description='ROS2 Humble package to control 16-channel Servo Driver HAT (PCA9685)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'servo_node = servo_controller.servo_node:main',
        ],
    },
)

