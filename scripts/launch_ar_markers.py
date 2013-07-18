import subprocess, os
import os.path as osp

ar_launch = osp.join(os.getenv('RAPPRENTICE_SOURCE_DIR'), 'launch', 'ar_tracker.launch')
FNULL = open(os.devnull, 'w')
subprocess.call("roslaunch " + ar_launch, shell=True, stdout=FNULL)