from setuptools import find_packages, setup

package_name = 'rm_serial_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='RM Team',
    maintainer_email='rm@example.com',
    description='ROS 2 串口桥接示例：将换行分隔的 UTF-8 文本与话题互相转发',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'serial_bridge = rm_serial_demo.serial_bridge:main',
        ],
    },
)
