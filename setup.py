from setuptools import setup, find_packages

setup(
    name='dobot_sim',
    version='0.1.0',  # バージョンを適宜変更
    packages=find_packages(),
    install_requires=[
        'numpy',
        'mujoco',
    ],
)