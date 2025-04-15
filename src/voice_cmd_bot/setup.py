from setuptools import setup

package_name = 'voice_cmd_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools', 'vosk', 'sounddevice'],
    zip_safe=True,
    maintainer='yourname',
    maintainer_email='you@example.com',
    description='Voice command node for ROS2',
   entry_points={
    'console_scripts': [
        'voice_node = voice_cmd_bot.voice_node:main',
        'command_mapper = voice_cmd_bot.command_mapper:main',
    ],
},
)
