from setuptools import find_packages, setup

package_name = 'map4_smartpole_detection_filter'

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
    maintainer='map4',
    maintainer_email='keisuke.urasaki@map4.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'area_filter = map4_smartpole_detection_filter.area_filter:main'
        ],
    },
)
