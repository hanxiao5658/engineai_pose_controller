from setuptools import find_packages, setup

package_name = 'engineai_pose_controller'

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
    maintainer='hanxiao',
    maintainer_email='hanxiao5658@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'global_pose_gamepad_controller = engineai_pose_controller.global_pose_gamepad_controller:main',
        'engineai_joint_state_to_standard = engineai_pose_controller.engineai_joint_state_to_standard:main',
        'engineai_base_pose_to_standard = engineai_pose_controller.engineai_base_pose_to_standard:main',
    ],
	},
)
