from setuptools import find_packages, setup

package_name = 'controller'

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
    maintainer='Gordon Fountain',
    maintainer_email='gordonthecellist@gmail.com',
    description='Package for controllers',
    license='None',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_node = controller.controller_node:main',
        ],
    },
)
