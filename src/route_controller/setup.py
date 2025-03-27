from setuptools import find_packages, setup

package_name = 'route_controller'

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
    maintainer='pentagon',
    maintainer_email='pentagon@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    # package_dir={'': 'route_controller'},
    # py_modules=['util'],
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driver = route_controller.drive:main',
            'runner = route_controller.twist_pub:main'
        ],
    },
)
