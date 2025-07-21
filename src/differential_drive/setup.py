from setuptools import setup

package_name = 'differential_drive'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='Differential drive node for robot PWM outputs',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'differential_drive_node = differential_drive.differential_drive_node:main',
        ],
    },
)
