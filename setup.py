from setuptools import setup

package_name = 'egh400_hmi_robotic_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='A ROS 2 package for EGH400 HMI Robotic Arm',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = egh400_hmi_robotic_arm.main:main',
        ],
    },
)
