from setuptools import setup

PACKAGE_NAME = 'rqt_finger_manipulation'

setup(
    name=PACKAGE_NAME,
    version='0.0.0',
    packages=[PACKAGE_NAME],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME + "/resource", ["resource/main_window.ui"]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        ("share/" + PACKAGE_NAME, ["plugin.xml"]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yuandi Huang',
    maintainer_email='yuandi@umich.edu',
    description='Custom rqt interface for finger_manipulation package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ["rqt_finger_manipulation = rqt_finger_manipulation.rqt_finger_manipulation:main"],
    },
)
