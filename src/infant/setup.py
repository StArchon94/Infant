import os
from glob import glob, iglob

from setuptools import find_packages, setup

package_name = 'infant'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ] + [(os.path.join('share', package_name, os.path.dirname(f)), [f]) for f in iglob('resource/**/*.*', recursive=True)],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shengjie Lin',
    maintainer_email='slin@ttic.edu',
    description='Infant is an interactive installation. This project artistically demonstrates an infant’s reaction to the external stimulation with the help of various information technologies, such as machine learning and digital signal processing.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ear = infant.infant_ear:main',
            'eye = infant.infant_eye:main',
            'pulse = infant.infant_pulse:main',
            'state_server = infant.infant_state_server:main',
            'touch = infant.infant_touch:main',
            'visualizer = infant.infant_visualizer:main',
        ],
    },
)
