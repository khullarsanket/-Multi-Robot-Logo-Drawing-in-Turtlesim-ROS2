from setuptools import find_packages, setup

package_name = 'project2'

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
    maintainer='lab2004',
    maintainer_email='lab2004@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'Go_to_Goal_Node = project2.working:main',
            'turtle_Go_to_Goal_Node = project2.sanket_test:main',
            'set_background_color_node = project2.sanket_test:main'
        ],
    },
)
