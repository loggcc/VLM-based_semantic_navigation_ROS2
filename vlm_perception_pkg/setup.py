from setuptools import find_packages, setup

package_name = 'vlm_perception_pkg'

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
    maintainer='ariel',
    maintainer_email='bhu16300@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'vlm_detection = vlm_perception_pkg.vlm_detection:main',
        'simple_detection = vlm_perception_pkg.simple_detection:run',  # function inside simple_detection.py
        'multi_waypoint_navigation = vlm_perception_pkg.simple_detection:main',  # function inside simple_detection.py
    ],
},




)
