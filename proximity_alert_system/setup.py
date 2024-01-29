from setuptools import setup

package_name = 'proximity_alert_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kaneko',
    maintainer_email='kaneko@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'alert_sender = proximity_alert_system.alert_sender:main',
            'detected_id_publisher = proximity_alert_system.detected_id_publisher:main',
        ],
    },
    package_data={
        package_name: ['msg/*.msg'],
    },
)
