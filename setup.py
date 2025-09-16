from setuptools import find_packages, setup

package_name = 'my_manipulator'

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
    maintainer='samiul',
    maintainer_email='samiulislamrudro75754@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node_creator=my_manipulator.node_creator:main',
            'object_detector=my_manipulator.object_detector:main',
            'kinematic_solver=my_manipulator.kinematic_solver:main',
            'trajectory_generator=my_manipulator.trajectory_generator:main'
            
        ],
    },
)
