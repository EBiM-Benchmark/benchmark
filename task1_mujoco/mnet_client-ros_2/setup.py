from setuptools import find_packages, setup
import os
from glob import glob
from pathlib import Path

package_name = 'mnet_client'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/team_config.json', 'config/banner.txt']),
    ]+ [
        # place files preserving relative paths under share/my_tasks_pkg/assets
        ("share/" + package_name + "/assets/" + str(p.parent.relative_to("assets")), [str(p)])
        for p in Path("assets").rglob("*") if p.is_file()
    ],
    install_requires=["requests>=2.32.2", "tqdm>=4.67.1", "pydantic>=2.0,<3.0"],
    zip_safe=True,
    maintainer='yitingchen',
    maintainer_email='yiting.chen@rice.edu',
    description='ManipulationNet: www.manipulation-net.org',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'submission = mnet_client.submission:main',
            'local_test = mnet_client.local_test:main',
            'connection_test = mnet_client.connection_test:main',
        ],
    },
)
