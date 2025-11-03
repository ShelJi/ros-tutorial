from setuptools import find_packages, setup

package_name = 'my_py_pkg'

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
    maintainer='shelj',
    maintainer_email='shelj.dev@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "my_minimal_publisher = my_py_pkg.minimal_publisher:main",
            "my_minimal_subscriber = my_py_pkg.minimal_subscriber:main",
            "my_minimal_server = my_py_pkg.minimal_server:main",
            "my_minimal_client = my_py_pkg.minimal_client:main",
        ],
    },
)
