from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vlm_nav_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add this line to install the config directory and its JSON files
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ariel',
    maintainer_email='bhu16300@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'point_nav = vlm_nav_pkg.point_nav:main',
        'semantic_nav = vlm_nav_pkg.semantic_nav:main' 
    ],
},
)
