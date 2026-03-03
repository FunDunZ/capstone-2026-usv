from setuptools import setup, find_packages
from glob import glob

package_name = 'usv_autonomy'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andrew Thomas',
    maintainer_email='you@example.com',
    description='USV autonomous navigation stack',
    license='MIT',
    entry_points={
        'console_scripts': [
            'detection_node = usv_autonomy.detection_node:main',
            'mission_node   = usv_autonomy.mission_node:main',
        ],
    },
)
