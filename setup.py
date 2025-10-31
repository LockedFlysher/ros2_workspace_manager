from setuptools import setup
import os
from glob import glob

package_name = 'workspace_manager'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, f'{package_name}.gui'],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('workspace_manager/config/*.yaml')),
        # 添加以下行来安装icon文件夹下的所有文件
        (os.path.join('share', package_name, 'icon'), glob('workspace_manager/icon/*.jpg')),
        # 安装样式表文件（QSS）
        (os.path.join('share', package_name, 'gui'), glob('workspace_manager/gui/*.qss')),
    ],
    install_requires=['setuptools', 'pyyaml'],  # 添加 pyyaml 依赖
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='ROS2 workspace manager with GUI',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'workspace_manager = workspace_manager.workspace_manager_node:main'
        ],
    },
)
