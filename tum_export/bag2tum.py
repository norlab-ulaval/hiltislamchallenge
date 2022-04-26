#!/usr/bin/env python
from __future__ import print_function

import argparse
from collections import OrderedDict
from os import path

import pandas as pd
import rosbag
from tqdm import tqdm

FIRST_ROW = "# timestamp_s tx ty tz qx qy qz qw"

class FilePath:
    def __init__(self, complete_path):
        self.folder, self.filename = path.split(complete_path)
        self.filestem, self.ext = path.splitext(self.filename)
        self.path = complete_path

    def __repr__(self):
        info = {
            "folder": self.folder,
            "name": self.filename,
            "stem": self.filestem,
            "ext": self.ext,
            "path": self.path,
        }
        return "{folder} | {name} | {stem} | {ext} | {path}".format(**info)

def parse_arguments():
    """Parse bagfile name"""
    parser = argparse.ArgumentParser()
    parser.add_argument("-b", "--bagfile", help="""Bagfile path""", required=True)
    parser.add_argument("-t", "--trajectory", help="""Trajectory topic""", default="/imu_trajectory_odom")
    args = parser.parse_args()
    bagfile = args.bagfile
    bag_filepath = FilePath(bagfile)
    return bag_filepath, args.trajectory


def decompose_msg(topic, msg):
    """Get important elements from a rosbag msg

    Args:
        topic (str): Topic of the msg
        msg (rosbag.msg): Structure that contains information at a certain time in the rosbag

    Raises:
        NotImplementedError: Topic is not implemented

    Returns:
        dict: Dictionary of important values from the msg
    """
    if topic == "/imu_trajectory_odom":
        timestamp = msg.header.stamp.to_sec()

        pose = msg.pose.pose
        posit = pose.position
        orient = pose.orientation

        return {
            "timestamp_s": timestamp,
            "tx": posit.x,
            "ty": posit.y,
            "tz": 0,
            "qx": orient.x,
            "qy": orient.y,
            "qz": orient.z,
            "qw": orient.w,
        }
    else:
        raise NotImplementedError("Topic %s is not implemented" % topic)



def get_topic_msgs(topic_, bag):
    """Get rosbag messages from a certain topic

    Args:
        topic_ (str): Topic of concern
        bag (rosbag.Bag): Bag object created with the `rosbag` library

    Returns:
        list: List of messages from the topic. Each message is given like a dictionary
    """
    data_list = []
    with tqdm(total=bag.get_message_count([topic_])) as pbar:
        for _, msg, t in bag.read_messages(topics=[topic_]):
            time_dat = decompose_msg(topic_, msg)
            # New csv row
            data_list.append(time_dat)
            # Progress bar
            pbar.update(1)

    return data_list

def tum_filename_format(bagstem):
    """Format tum file name according to bag file name"""
    return bagstem.rsplit("_odom")[0]
    # return bagstem

def export_to_tum(topic, msg_arr, bag_path):
    df = pd.DataFrame(msg_arr)
    print("Columns :", df.columns)
    df = df[FIRST_ROW[2:].split()]

    # Create a folder for CSV
    traj_path = path.join(bag_path.folder.rsplit("/odom")[0], "trajectories")
    if not path.exists(traj_path):
        import os

        os.makedirs(traj_path)

    # Export df to CSV
    tum_path = path.join(
        traj_path, "%s.txt" % tum_filename_format(bag_path.filestem)
    )
    print("Saving to", tum_path)
    with open(tum_path, "w") as f:
        f.write(FIRST_ROW + "\n")
    df.to_csv(tum_path, index=False, header=False, mode="a", sep=" ")



def main():
    bagfp, topic = parse_arguments()
    with rosbag.Bag(bagfp.path) as bag:
        messages = get_topic_msgs(topic, bag)
        export_to_tum(topic, messages, bagfp)

if __name__ == "__main__":
    main()

