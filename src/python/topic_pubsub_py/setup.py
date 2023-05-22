from setuptools import setup

package_name = 'topic_pubsub_py'

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
    maintainer='Roberto Masocco',
    maintainer_email='robmasocco@gmail.com',
    description='Topic publisher/subscriber example.',
    license='GPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sub = topic_pubsub_py.sub:main',
            'pub = topic_pubsub_py.pub:main'
        ],
    },
)
