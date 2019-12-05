from setuptools import setup

package_name = 'fake_point_cloud'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sloretz',
    maintainer_email='sloretz@openrobotics.org',
    description='Publish fake PointCloud2 message',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_point_cloud = fake_point_cloud.fake_point_cloud:main'
        ],
    },
)
