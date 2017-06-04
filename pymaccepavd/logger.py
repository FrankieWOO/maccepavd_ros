#! /usr/bin/env python

import rospy, rosbag, sys, os, numpy, subprocess, math


class Logger(object):
    """Logger: log messages of robto state and command using rosbag and do
    some auto data processing."""
    bagfile = os.path.dirname(os.path.abspath(__file__)) + '/log/templog.bag'
    def __init__(self, bagfile, output_filename):
        super(Logger, self).__init__()
        self.output_filename = output_filename
        self.bagfile = bagfile

    def start(self):
        """start rosbag writing in a while loop"""
        bag = rosbag.Bag(self.bagfile, 'w')
        while condition:
            pass

    def stop(self):
        bag.close()
        timenow = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        subprocess.call('rostopic echo -p -b {} /sensors > {}'.format(
        self.bagfile, output_filename), shell = True)
