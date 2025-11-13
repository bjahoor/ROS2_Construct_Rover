from setuptools import find_packages, setup

package_name = 'services_quiz'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/services_quiz_server.launch.py',
            'launch/services_quiz_client.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='bjahoor@uwaterloo.ca',
    description='Server and client for /turn service',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turn_s_server = services_quiz.turn_s_server:main',
            'turn_s_client = services_quiz.turn_s_client:main',
        ],
    },
)
