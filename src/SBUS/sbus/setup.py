from setuptools import find_packages, setup

package_name = 'sbus'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kyprus',
    maintainer_email='milkomeda@seoultech.ac.kr',
    description='get sbus signals from UART',
    license='None',
    
    entry_points={
        'console_scripts': [
            'sbus_worker = sbus.sbus_node:main'
        ],
    },
)
