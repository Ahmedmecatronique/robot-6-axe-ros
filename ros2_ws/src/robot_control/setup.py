from setuptools import setup

package_name = 'robot_control'

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
    description='Sécurité, simulateur et saisie du bras 6 axes',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'safety_node = robot_control.safety_node:main',
            'sim_robot_node = robot_control.sim_robot_node:main',
            'pick_node = robot_control.pick_node:main',
            'bag_node = robot_control.bag_node:main',
        ],
    },
)
