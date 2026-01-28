from setuptools import find_packages, setup

package_name = 'turtlebot4_depth'

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
    maintainer='rokey',
    maintainer_email='dlwnsh925@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'depth_checker = turtlebot4_depth.turtlebot4_depth_checker:main',
            'depth_checker_mouse_click = turtlebot4_depth.turtlebot4_depth_checker_mouse_click:main',
        ],
    },
)
