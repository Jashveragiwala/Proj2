from setuptools import find_packages, setup
import os
import glob

package_name = 'sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), glob.glob('share/*')),
        (os.path.join('share', package_name), ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jokane',
    maintainer_email='jokane@tamu.edu',
    description='Tiny simluators for CSCE452/752.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim0 = sim.sim0:main',
            'sim1 = sim.sim1:main',
            'circle = circle.circle_drawing:main',
         ],
    },
)
