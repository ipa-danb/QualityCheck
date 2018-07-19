#!/usr/bin/env python
import rospy
from geometry_msgs.msg import WrenchStamped
from robot_movement_interface.msg import EulerFrame
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger
from denso.srv import startForceLog


import os
import sys
import threading
import pickle
import datetime
import time
import pandas as pd
import pyarrow as pa
import numpy as np
import pyarrow.parquet as pq

class Analyzer(object):
    def __init__(self, arg):
        self.datadir = '/home/dragandbot/data'
        self.workdir = os.

        pass

    def configFile(self,taskname):


    def findstuff(self,index,stuff):
        for i,el in enumerate(index.levels):
            for el2 in el:
                if stuff in el2:
                    return True,el2,i

        return False,''

    def extractStats(self,fileName,selector):
        df = pq.read_table(fileName).to_pandas()
        aw = findstuff(df.index,selector)
        df2 = df.reset_index()
        ar = df2.loc[df2['level_'+str(aw[2])] == aw[1]][['xf','yf','zf']]
        return list(ar.mean()), list(ar.std())

    def handle_callback(self,req):
        pass

    def loadData(self,taskname):

    def listener(self):
        rospy.init_node('analyzer', anonymous=True)

        # TODO give those meaningful names
        #rospy.Service('/forcelogger/startForceLog', startForceLog, self.start_forcelog)
        #rospy.Service('/forcelogger/stopForceLog', Trigger, self.stop_forcelog)

        # spin() simply keeps python from exiting until this node is stopped
        try:
            rospy.spin()
        finally:
            rospy.logwarn("Shutting down analyzer node")

if __name__ == '__main__':
