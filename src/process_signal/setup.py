from setuptools import find_packages, setup

package_name = 'process_signal'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/signal_launch.py']),
        ('share/' + package_name + '/config', ['config/params.yaml']),

    ],
    install_requires=['setuptools', 'signal_msg'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='a00833638@tec.mx',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'signal_generator = process_signal.signal_generator:main',
            'signal_reconstruction = process_signal.signal_reconstruction:main'

        ],
    },
)
