from setuptools import setup

package_name = 'my_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/demo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abdelrhaman',
    maintainer_email='abdelrahman.y.salah@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_turtle_control = my_package.base_turtle_control:main',
           
        ],
    },
)