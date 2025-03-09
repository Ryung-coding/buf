from setuptools import find_packages, setup
import os

package_name = 'mujoco_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', package_name, 'xml'), [
            'xml/scene.xml',
            'xml/quadrotor.xml'
        ]),
        (os.path.join('share', package_name, 'xml/assets'), [
            'xml/assets/X2_lowpoly.obj',
            'xml/assets/X2_lowpoly_texture_SpinningProps_1024.png'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kyprus',
    maintainer_email='milkomeda@seoultech.ac.kr',
    description='drone simulator node with mujoco',
    license='None',
    
    entry_points={
        'console_scripts': [
            'mujoco_node = mujoco_sim.mujoco_sim:main',
        ],
    },
)
