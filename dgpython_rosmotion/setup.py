from setuptools import setup
import os
from glob import glob

package_name = 'dgpython_rosmotion'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'demos'), glob('demos/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Tyler Lesthaeghe',
    maintainer_email='tyler.lesthaeghe.ctr@us.af.mil',
    description='Basic ROS2 Motion Package using TF2 and dgpython_ros2_actionserver',
    license='BSD-3-Clause',
    tests_require=['pytest']
)
