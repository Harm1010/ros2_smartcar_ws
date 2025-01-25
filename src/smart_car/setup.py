from setuptools import setup

pkg_name = 'smart_car'  # Define the package name

# Configure the setup
setup(
    name=pkg_name,
    version='0.0.0',
    packages=[pkg_name],
    py_modules=[],  # No standalone Python modules outside the package directory
    data_files=[
        # Add the package to the ROS 2 ament index
        ('share/ament_index/resource_index/packages', 
         ['resource/' + pkg_name]),
        # Include the package.xml in the share directory
        ('share/' + pkg_name, ['package.xml']),
    ],
    install_requires=['setuptools'],  # Setuptools as a dependency
    zip_safe=True,
    maintainer='Harm Smits',  # Maintainer information
    maintainer_email='HS.Smits@student.han.nl',
    description='A hybrid package with both C++ and Python nodes',  # Package description
    license='Apache License-2.0',  # License type
    tests_require=['pytest'],  # Dependencies for testing
    entry_points={
        'console_scripts': [
            # Define console scripts here if any
        ],
    },
)

