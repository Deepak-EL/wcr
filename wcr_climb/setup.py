from setuptools import setup

package_name = 'wcr_climb'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Girish',
    maintainer_email='booyahmaster8i@gmail.com',
    description='EDF adhesion controller and climb tests for the WCR on the dam face',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'adhesion_controller = wcr_climb.adhesion_controller:main',
            'climb_test = wcr_climb.climb_test:main',
            'wait_for_sim = wcr_climb.wait_for_sim:main',
        ],
    },
)
