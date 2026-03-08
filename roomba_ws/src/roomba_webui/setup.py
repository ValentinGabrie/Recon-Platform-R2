from setuptools import setup, find_packages

package_name = 'roomba_webui'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    package_data={
        package_name: [
            'templates/*.html',
            'static/css/*.css',
            'static/js/*.js',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gabi',
    maintainer_email='gabi@roomba.local',
    description='Flask web server and WebSocket bridge for roomba',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'roomba_webui = roomba_webui.app:main',
        ],
    },
)
