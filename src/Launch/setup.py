from setuptools import setup, find_packages

package_name = 'jfish'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='.'),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jinu',
    maintainer_email='milkomeda@seoultech.ac.kr',
    description='Launch package for managing all nodes',
    license='None',
    entry_points={
        'console_scripts': [],
    },
)