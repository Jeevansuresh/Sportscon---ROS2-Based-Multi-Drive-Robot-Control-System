from setuptools import setup

package_name = 'mecanum_drive'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='Mecanum drive with joystick and keyboard control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystick_node = mecanum_drive.joystick_node:main',
            'keyboard_node = mecanum_drive.keyboard_node:main',
            'mecanum_node = mecanum_drive.mecanum_node:main',
        ],
    },
)
