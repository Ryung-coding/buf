from setuptools import setup

package_name = 'pid_debugger'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kyprus',
    maintainer_email='kyprus@example.com',
    description='PID Debugger for monitoring controller and allocator info',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pid_debugger = pid_debugger.pid_debugger:main',
        ],
    },
)