from setuptools import find_packages, setup

package_name = 'odom_republisher'

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
    maintainer='astra',
    maintainer_email='astra@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "odom_republisher_node = odom_republisher.odom_republisher_node:main",
            "set_origin = odom_republisher.set_origin:main",
        ],
    },
)
