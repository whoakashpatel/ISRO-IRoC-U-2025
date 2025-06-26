from setuptools import find_packages, setup

package_name = 'survey_and_nav'

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
    maintainer='hsaka',
    maintainer_email='akash.patel9690@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'survey_node = survey_and_nav.mission:main',
            'testing_node = survey_and_nav.testing:main'
        ],
    },
)
