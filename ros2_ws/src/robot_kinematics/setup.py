from setuptools import setup

package_name = 'robot_kinematics'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ahmed',
    maintainer_email='ahmedetude21@gmail.com',
    description='Cinématique FK/IK, trajectoire et nœuds du bras 6 axes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fk_node = robot_kinematics.fk_node:main',
            'ik_node = robot_kinematics.ik_node:main',
            'trajectory_node = robot_kinematics.trajectory_node:main',
        ],
    },
)
