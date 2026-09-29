from setuptools import setup

package_name = 'ue_bridge_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Timofey',
    maintainer_email='kudakovt41@gmail.com',
    description=(
        'Мост между UE5-симулятором и ROS2-пайплайном metro_obstacle_detection. '
        'Только для разработки/теста/визуализации, не часть сдаваемого решения.'
    ),
    license='Unlicense',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ue_bridge_node = ue_bridge_pkg.ue_bridge_node:main',
            'object_classifier_node = ue_bridge_pkg.object_classifier_node:main',
        ],
    },
)
