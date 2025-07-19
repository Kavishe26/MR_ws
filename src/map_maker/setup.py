from setuptools import find_packages, setup

package_name = 'map_maker'

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
    maintainer='kavishe',
    maintainer_email='kavishe@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "path_planning_node = map_maker.path_planning_node : main" ,
            "explorer_node = map_maker.explorer_node : main",
            "FSM_controller_node = map_maker.FSM_controller_node : main"
        ],
    },
)
