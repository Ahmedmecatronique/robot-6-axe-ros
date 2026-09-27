from setuptools import setup

package_name = 'robot_gui'

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
    description='IHM du bras 6 axes (jumeau numérique)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gui = robot_gui.main:main',
        ],
    },
)
