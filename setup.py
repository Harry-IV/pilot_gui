from setuptools import find_packages, setup

package_name = 'pilot_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/camera_ips.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jhsrobo',
    maintainer_email='ohaginh26@student.jhs.net',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_viewer = pilot_gui.camera_viewer:main',
            'test_controller = pilot_gui.test_controller:main',
            'overlay_camera_viewer = pilot_gui.camera_viewer_with_overlay:main'
        ],
    },
)
