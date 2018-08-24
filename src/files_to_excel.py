#!/usr/bin/python

import os
import glob
import pandas as pd
import argparse
import rospy


parser = argparse.ArgumentParser(description='Converts logs to excel files')
parser.add_argument('-d', '--directory', type=str, help='direcotry of logfiles')
parser.add_argument('-v', '--verbose', action="store_true", help="verbose mode")

try:
	args = parser.parse_args(rospy.myargv()[1:]) # from ROS there could be arguments which interfere with argparse
	directory = args.directory
except:
	directory = os.getcwd()

fileList = [f for f in glob.glob(os.path.join(directory,'*')) if '.xlsx' not in f and '.py' not in f ]

for element in fileList:
	if args.verbose:
		print("Try to convert {0}".format(os.path.basename(element)))
	try:
		pd.read_parquet(element).to_excel(element + '.xlsx')
		if args.verbose:
			print("Success!")
	except:
		if args.verbose:
			print("... failed! Skipping...")
