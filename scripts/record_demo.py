#!/usr/bin/env python

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("demo_prefix")
parser.add_argument("master_file")
parser.add_argument("--annotation_func", default="suture_gen")
parser.add_argument("--downsample", default=3, type=int)
args = parser.parse_args()

import subprocess, signal
from rapprentice.colorize import colorize
import time, os, shutil
from rapprentice.call_and_print import call_and_print
from rapprentice.yes_or_no import yes_or_no
import os.path as osp
import itertools
import yaml

import rospy

started_bag = False
started_video = False

localtime   = time.localtime()
time_string  = time.strftime("%Y-%m-%d-%H-%M-%S", localtime)

if not osp.isfile(args.master_file):
    master_name = args.master_file.split("/")[-1]
    if master_name[-5:] == '.yaml':
        master_name = master_name.split(".")[-2] 
    with open(args.master_file, "w") as f:
        f.write("name: %s\n"%master_name)
        f.write("h5path: %s\n"%(master_name+".h5"))
        f.write("bags: ")

dirname = osp.dirname(args.master_file)
os.chdir(dirname)

with open(args.master_file, "r") as fh: master_info = yaml.load(fh)
if master_info["bags"] == None:
    demo_name = args.demo_prefix
else:
    for suffix in itertools.chain("", (str(i) for i in itertools.count())):
        demo_name = args.demo_prefix + suffix
    
        if not any(bag["demo_name"] == demo_name for bag in master_info["bags"]):
            break
        print demo_name

rospy.init_node('record_demo')
subprocess.call("killall XnSensorServer", shell=True)

try:

    bag_cmd = "rosbag record /joint_states /joy -O %s"%demo_name
    print colorize(bag_cmd, "green")
    bag_handle = subprocess.Popen(bag_cmd, shell=True)
    started_bag = True
    
    video_cmd = "record_rgbd_video --out=%s --downsample=%i"%(demo_name, args.downsample)
    print colorize(video_cmd, "green")
    video_handle = subprocess.Popen(video_cmd, shell=True)
    started_video = True
    
    time.sleep(9999)    

except KeyboardInterrupt:
    print colorize("got control-c", "green")

finally:
    
    if started_bag:
        bag_handle.send_signal(signal.SIGINT)
        bag_handle.wait()
    if started_video:
        video_handle.send_signal(signal.SIGINT)
        video_handle.wait()


bagfilename = demo_name+".bag"
if yes_or_no("save demo?"):
    annotations_gen_file = osp.join(os.getenv('RAPPRENTICE_SOURCE_DIR'), 'scripts', 'generate_annotations.py')
    annfilename = demo_name+".ann.yaml"
    call_and_print(annotations_gen_file + " %s %s --annotation_func=%s"%(osp.join(dirname, bagfilename), osp.join(dirname, annfilename), args.annotation_func))
    with open(args.master_file,"a") as fh:
        fh.write("\n"
            "- bag_file: %(bagfilename)s\n"
            "  annotation_file: %(annfilename)s\n"
            "  video_dir: %(videodir)s\n"
            "  demo_name: %(demoname)s"%dict(bagfilename=bagfilename, annfilename=annfilename, videodir=demo_name, demoname=demo_name))
else:
    shutil.rmtree(demo_name) #video dir
    os.unlink(bagfilename)
