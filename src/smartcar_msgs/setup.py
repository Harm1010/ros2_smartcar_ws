from setuptools import setup

# Package details
package_name = 'hybrid_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Installation paths for package indexing and XML metadata
        (f'share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
    ],
    # Dependencies
    install_requires=['setuptools'],
    zip_safe=True,
    
    # Maintainer information
    maintainer='Harm Smits'
    maintainer_email='HS.Smits@student.han.nl',
    
    # Package metadata
    description='A hybrid package supporting both C++ and Python nodes',
    license='TODO: License declaration',
    tests_require=['pytest'],
    
    # Entry points for executable scripts
    entry_points={
        'console_scripts': [
            'python_node = hybrid_pkg.python_node:main',
        ],
    },
)

