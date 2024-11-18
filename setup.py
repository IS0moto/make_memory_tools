from setuptools import find_packages, setup

package_name = 'make_memory_tools'

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
    maintainer='d2302',
    maintainer_email='kak31307465@icloud.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'compressedImage2dataset = make_memory_tools.compressedImage2dataset:main',
            'compressedImage2video = make_memory_tools.compressedImage2video:main',
            'getPlan = make_memory_tools.get_plan:main'
        ],
    },
)
