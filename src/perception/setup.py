from setuptools import find_packages, setup

package_name = 'perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/perception_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='razanhmede',
    maintainer_email='razanhmede@lau.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'Blackband_detector = perception.Blackband_detector:main',
            'Blue_detector = perception.Blue_detector :main',
            'White_detector = perception.White_detector:main',
        ],
    },
)


