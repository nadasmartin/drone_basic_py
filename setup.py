from setuptools import find_packages, setup
from glob import glob

package_name = 'drone_basic_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
        (f'share/{package_name}/urdf', glob('urdf/*')),
        (f'share/{package_name}/rviz', glob('rviz/*')),
        (f'share/{package_name}/worlds', glob('worlds/*')),
        (f'share/{package_name}/config', glob('config/*')),
        (f'share/{package_name}/meshes', glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mibalazs',
    maintainer_email='mibalazs@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
