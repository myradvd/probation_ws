from setuptools import setup

package_name = 'task'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='Mapless AUV gate-passing navigation stack',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mode_switcher = task.mode_switcher:main',
            'depth_controller = task.depth_controller:main',
            'nav_controller = task.nav_controller:main',
            'nav_debugger = task.nav_debugger:main',
        ],
    },
)