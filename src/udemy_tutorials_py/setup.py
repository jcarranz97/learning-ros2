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
        ],
    },
)
