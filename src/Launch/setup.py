from setuptools import setup, find_packages

package_name = 'launch_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(where='.'),  # 현재 디렉토리에서 탐색
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Launch package for managing all nodes',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)