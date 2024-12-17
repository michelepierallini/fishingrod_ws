from setuptools import find_packages, setup

package_name = 'ddp_fishing'

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
    maintainer='michele',
    maintainer_email='michele.pierallini@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ddp_controller_publisher_node = ddp_fishing.ddp_controller_publisher_node:main',
            'bag_recorder = ddp_fishing.bag_recorder:main'
        ],
    },
)
