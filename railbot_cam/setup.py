from setuptools import setup

package_name = 'railbot_cam'

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
    maintainer='dan',
    maintainer_email='goldsounds@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'img_publisher = railbot_cam.webcam_pub:main',
            'img_subscriber = railbot_cam.webcam_sub:main',
			'img_identifier = railbot_cam.webcam_identify:main',
			'person_subscriber = railbot_cam.person_sub:main',
        ],
    },
)
