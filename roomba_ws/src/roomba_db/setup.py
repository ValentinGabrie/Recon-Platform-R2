from setuptools import setup

package_name = 'roomba_db'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gabi',
    maintainer_email='gabi@roomba.local',
    description='Database abstraction layer for roomba',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'db_node = roomba_db.db_node:main',
        ],
    },
)
