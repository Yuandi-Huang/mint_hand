from setuptools import find_packages, setup

package_name = 'finger_manipulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['rqt_plugin', 'rqt_plugin.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yuandi Huang',
    maintainer_email='yuandi@umich.edu',
    description='Graphical interface',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rqt_plugin = rqt_plugin.plugin:main'
        ],
    },
)
