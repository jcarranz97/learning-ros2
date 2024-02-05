from setuptools import find_packages, setup

package_name = 'udemy_tutorials_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'metadata'],
    zip_safe=True,
    maintainer='juan',
    maintainer_email='juancarranza7561@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "py_node = udemy_tutorials_py.my_first_node:main",
            "robot_news_station = udemy_tutorials_py.robot_news_station:main",
            "smartphone = udemy_tutorials_py.smartphone:main",
            "number_publisher = udemy_tutorials_py.number_publisher:main",
            "number_counter = udemy_tutorials_py.number_counter:main",
            "add_two_ints_server = udemy_tutorials_py.add_two_ints_server:main",
            "add_two_ints_client_no_oop = udemy_tutorials_py.add_two_ints_client_no_oop:main",
            "add_two_ints_client_oop = udemy_tutorials_py.add_two_ints_client_oop:main",
            "hw_status_publisher = udemy_tutorials_py.hw_status_publisher:main",
            "compute_rectangle_area_server = udemy_tutorials_py.compute_rectangle_area_server:main",
            "battery_node = udemy_tutorials_py.battery_node:main",
        ],
    },
)
