from setuptools import setup
from setuptools import find_packages

setup(
    name='insta360_ros2_driver',
    version='0.0.1',  # Define your package version
    packages= find_packages('src'),
    package_dir={'': 'src'},
    install_requires=['setuptools', 'rclpy', 'cv_bridge'],
)