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
            'xml/Arm1_Link1.STL',
            'xml/Arm1_Link2.STL',
            'xml/Arm1_Link3.STL',
            'xml/Arm1_Link4.STL',
            'xml/Arm1_Link5.STL',
            'xml/Arm2_Link1.STL',
            'xml/Arm2_Link2.STL',
            'xml/Arm2_Link3.STL',
            'xml/Arm2_Link4.STL',
            'xml/Arm2_Link5.STL',
            'xml/Arm3_Link1.STL',
            'xml/Arm3_Link2.STL',
            'xml/Arm3_Link3.STL',
            'xml/Arm3_Link4.STL',
            'xml/Arm3_Link5.STL',
            'xml/Arm4_Link1.STL',
            'xml/Arm4_Link2.STL',
            'xml/Arm4_Link3.STL',
            'xml/Arm4_Link4.STL',
            'xml/Arm4_Link5.STL',
            'xml/JellyFish.xml',
            'xml/scene.xml',
            'xml/BODY.STL'
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
