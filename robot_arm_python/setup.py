from setuptools import setup

package_name = 'robot_arm_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    zip_safe=True,
    author='Juan Miguel Jimeno',
    author_email='jimenojmm@gmail.com',
    maintainer='Juan Miguel Jimeno',
    maintainer_email='jimenojmm@gmail.com',
    description='Robot Arm Python Library',
    license='Apache 2.0',
)
