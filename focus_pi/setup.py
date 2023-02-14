from setuptools import setup

package_name = 'focus_pi'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hideonbush',
    maintainer_email='hideonbush@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_pi = focus_pi . image_pi : main',
            'solve_pi   = focus_pi . solve_pi : main',
            'serial_pi  = focus_pi  . stepper_motor_pi :main'
        ],
    },
)
