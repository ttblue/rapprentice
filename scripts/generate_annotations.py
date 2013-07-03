#!/usr/bin/env python

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("bagfile")
parser.add_argument("outfile")
parser.add_argument("--annotation_func", default="normal_gen")
args = parser.parse_args()
from rapprentice import bag_proc

import yaml, rosbag

def normal_gen(bagfile, outfile):
    bag = rosbag.Bag(args.bagfile)
    stamps, meanings = bag_proc.extract_joy(bag)
    print "joystick button presses:"
    for (stamp, meaning) in zip(stamps, meanings):
        print "\t%.4f: %s"%(stamp/1.e9, meaning)
    
    seg_infos = bag_proc.joy_to_annotations(stamps, meanings)
    for (i_seg, seg_info) in enumerate(seg_infos): 
        seg_info["name"] = "seg%.2i"%i_seg
        seg_info["description"] = "(no description)"
    print "segment info: "
    print seg_infos
    
    
    
    print "writing to %s"%args.outfile
    with open(args.outfile,"w") as fh:
        yaml.dump(seg_infos,fh)
        

def suture_gen(bagfile, outfile):
    bag = rosbag.Bag(args.bagfile)
    stamps, meanings = bag_proc.extract_joy(bag)
    print "joystick button presses:"
    for (stamp, meaning) in zip(stamps, meanings):
        print "\t%.4f: %s"%(stamp/1.e9, meaning)

    video_dir = bagfile.split(".")[-2]
    import os.path as osp
    import suturing.find_keypoints as fk
    seg_infos = fk.create_annotations(stamps, meanings, bagfile, video_dir)
    
    for (i_seg, seg_info) in enumerate(seg_infos): 
        seg_info["name"] = "seg%.2i"%i_seg
        seg_info["description"] = "(no description)"
    print "segment info: "
    print seg_infos
    
    
    print "writing to %s"%args.outfile
    with open(args.outfile,"w") as fh:
        yaml.dump(seg_infos,fh)


func = globals().get(args.annotation_func)
if not func:
    raise NotImplementedError("The function %s was not implemented."%args.annotation_func)
else:
    func(args.bagfile, args.outfile)