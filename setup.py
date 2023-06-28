from setuptools import setup

package_name = 'crtk'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, glob('scripts/*py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anton Deguet',
    maintainer_email='anton.deguet@jhu.edu',
    description='CRTK Python client for ROS2',
    license='MIT',
    entry_points={
        'console_scripts': [
        ],
    },
)
