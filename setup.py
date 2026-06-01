from setuptools import find_packages, setup

package_name = 'lab4_aruco_exercise'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@example.com',
    description='Lab 4: ArUco marker mapping node.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_map_marker = lab4_aruco_exercise.aruco_map_marker:main',
            'basic_aruco_cv_transformation = lab4_aruco_exercise.basic_aruco_cv_transformation:main',
        ],
    },
)
