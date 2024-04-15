#!/home/administrator/anaconda3/envs/maia_ws/bin python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
import torch
setup_args = generate_distutils_setup(
    packages=[
        'yolov7_backend',
        'mobile_net_backend'
    ],
    package_dir={'': 'src'}
)
setup(**setup_args)