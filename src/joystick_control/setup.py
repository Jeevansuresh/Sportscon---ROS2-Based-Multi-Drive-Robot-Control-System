from setuptools import setup

package_name = 'joystick_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='skyno',
    maintainer_email='jeevansuresh258@email.com',
    description='Joystick control node for differential drive robot',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystick_control_node = joystick_control.joystick_control_node:main',
            'keyboard_control_node = joystick_control.keyboard_control_node:main',
        ],
    },
)
