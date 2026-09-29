from glob import glob
from pathlib import Path
from setuptools import find_packages, setup

setup(
    name='ur3_llm_control', version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/ur3_llm_control']),
        ('share/ur3_llm_control', ['package.xml', 'README.md']),
        *[('share/ur3_llm_control/' + folder, [p for p in glob(folder + '/*') if Path(p).is_file()])
          for folder in ['launch', 'config', 'prompts', 'docs']],
    ],
    install_requires=['setuptools', 'PyYAML'], zip_safe=True,
    maintainer='Ngo Thien Dac', maintainer_email='student@example.com',
    description='LLM -> validated skills -> MoveIt 2 -> UR3 in Gazebo Fortress',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'llm_planner_node = ur3_llm_control.planner_node:main',
        'command = ur3_llm_control.command:main',
    ]},
)
