from setuptools import setup

package_name = 'turtlebot3_automation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lenovo',
    maintainer_email='lenovo@todo.todo',
    description='TurtleBot3 automation package with maintenance, navigation, detection, and safety features',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'maintenance_node = turtlebot3_automation.maintenance_node:main',
            'navigation_manager = turtlebot3_automation.navigation_manager:main',
            'object_detection_node = turtlebot3_automation.object_detection_node:main',
            'safety_stop_node = turtlebot3_automation.safety_stop_node:main',
        ],
    },
)
