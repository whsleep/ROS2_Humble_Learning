from setuptools import find_packages, setup

package_name = 'publisher'

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
    maintainer='whf',
    maintainer_email='2035387001@qq.com',
    description='TODO: Package description',
    license='MIT-0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pub_node = publisher.status_pub:main',
        ],
    },
)
