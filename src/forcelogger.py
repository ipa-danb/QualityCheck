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
#import msgpack as pickle
import pickle
import datetime
import time
from collections import OrderedDict as odict
import pandas as pd
import pyarrow as pa
import numpy as np
import pyarrow.parquet as pq

class Forcelogger:
    def __init__(self,**kwargs):
        try:
            if kwargs['filename'] is None:
                self.data_file_name = "data"
            else:
                self.data_file_name = kwargs['filename']
        except KeyError:
            self.data_file_name = "data"

        try:
            self.breaksize = int(kwargs['breaksize'])
        except:
            self.breaksize = 20

        rospy.loginfo("Using maximum cache of ", self.breaksize, " MB")

        try:
            path = os.path.expanduser('~')
            if os.path.isdir(kwargs['directory']) and kwargs['directory'] is not None:
                self.directory = os.path.join(path,kwargs['directory'])
            else:
                self.directory = os.path.join(path,"data")
        except:
            self.directory = os.path.join(path,"data")

        rospy.loginfo("Using path: ", self.directory)

        # Initialize logging lists
        self.timings = []
        self.list_training_data = [[],[]]
        self.current_states = []
        self.start_times = []
        self.last_measurement = {'xp': 0,'yp': 0,'zp': 0,'ap': 0,'bp': 0,'gp': 0,'timeP':0}
        self.start_time = 0
        self.counter= 0

        # Initialize Lock
        self.lock = threading.Lock()

    ###################################
    ## Handles for service callbacks
    ###################################

    def start_forcelog(self,req):
        rospy.loginfo("Start Forcelog with filename: " + req.name)
        self.data_file_name = req.name
        self.clearLog()
        self.subscribe()
        return True

    def stop_forcelog(self,req):

        try:
            self.unsubscribe()
            self.saveLog()
            self.clearLog()
            rospy.loginfo("Stopping Forcelog")
            rospy.loginfo("---")
            return True, ''
        except Exception as ex:
            return False, str(ex)

    def handle_clear(self,req):
        """Clears datalog
        """
        try:
            self.unsubscribe()
            self.clearLog()
        except Exception as ex:
            return False, str(ex)

    ###################################
    ## Handles for service callbacks
    ###################################

    def saveLog(self):
        # Create File Name with timestring
        timestr = time.strftime("%Y%m%d-%H%M%S")
        file_name = '{0}_{1}'.format(self.data_file_name,timestr)

        ###############
        # paquet with brotli compression
        ###############
        start_time = time.time()
        if not self.list_training_data:
            rospy.loginfo("No Data to save")
            return False
        try:
            # Parquet with Brotli compression
            with self.lock:
                length = len(sorted(self.list_training_data[1],key=len, reverse=True)[0])
                preind = [tuple(xi+['-']*(length-len(xi))) for xi in self.list_training_data[1]]
                mltind = pd.MultiIndex.from_tuples(preind)
                df2 = pd.DataFrame(self.list_training_data[0],index=mltind)
                table =  pa.Table.from_pandas(df2)
                pq.write_table(table, os.path.join(self.directory,file_name), compression='BROTLI')
                rospy.loginfo("Saved file as %s", file_name)
        except Exception as ex:
            rospy.logerr(ex)

        end_time = time.time()

        rospy.loginfo("Time needed for saving: {0:.2f}s".format(end_time- start_time))

    def clearLog(self):
        self.list_training_data = [[],[]]
        rospy.loginfo("Cleared datalog")

    ###################################
    ## Subscribe & Unsubscribe functions
    ###################################
    def subscribe(self):
        rospy.loginfo("Subscribing to /tf_chain and /wrench")
        self.sub_tf_chain = rospy.Subscriber("/tf_chain",EulerFrame,self.callback_tfchain)
        self.sub_wrench = rospy.Subscriber("/wrench", WrenchStamped, self.callback_wrench)

    def unsubscribe(self):
        rospy.loginfo("Unsubscribing from /tf_chain and /wrench")
        try:
            self.sub_tf_chain.unregister()
        except:
            rospy.loginfo("TF Chain subscriber didnt exist")
        try:
            self.sub_wrench.unregister()
        except:
            rospy.loginfo("Wrench subscriber didnt exist")

    ###################################
    ## Callbacks
    ###################################

    def callback_tfchain(self,data):
        """
        # This message follows the Euler Intrinsic ZYX convention:
        # x,y,z in meters
        # - First alpha rads are rotated in Z axis
        # - Then beta rads are rotated in the new Y axis
        # - Finally gamma rads are rotated in the new X axis
        float32 x
        float32 y
        float32 z
        float32 alpha
        float32 beta
        float32 gamma
        """
        self.counter += 1
        with self.lock:
            tme2 =  rospy.Time.now().to_sec()
            self.last_measurement = {'xp': data.x,
                                     'yp': data.y,
                                     'zp': data.z,
                                     'ap': data.alpha,
                                     'bp': data.beta,
                                     'gp': data.gamma,
                                     'timeP':tme2}
            if self.counter % 3000 == 0:
                self.counter = 0
                if sys.getsizeof(self.list_training_data)/(1024.0*1024.0) > self.breaksize:
                    rospy.logwarn("Size of Log over {0} MB, save data, stopping Logger".format(self.breaksize))
                    self.unsubscribe()
                    self.saveLog()
                    self.clearLog()
                    #print("Size of Log: {0} MB".format(sys.getsizeof(self.list_training_data)/(1024.0*1024.0)))

    # This function is called whenever a wrench data set arrives
    def callback_wrench(self,data):
        tme = rospy.Time.now().to_sec()
        with self.lock:
            for state in self.current_states:
                idx = self.current_states.index(state)
                start_time = self.start_times[idx]
                self.start_time = start_time

                meas = {'xf': data.wrench.force.x,
                        'yf': data.wrench.force.y,
                        'zf': data.wrench.force.z,
                        'xt': data.wrench.torque.x,
                        'yt': data.wrench.torque.y,
                        'zt': data.wrench.torque.z,
                        'timeF': tme - start_time}

                meas.update(self.last_measurement)

                meas['timeP'] = meas['timeP'] - start_time
                try:
                    self.list_training_data[0].append(meas)
                    self.list_training_data[1].append(list(self.current_states))
                except:
                    print("Topic doesnt exist")

    # This function is called whenever a information about the active state arrives
    def callback_log(self,data):
        tme = rospy.Time.now().to_sec()
        with self.lock:
            # Checks if a new state is entered
            if data.data.startswith("Entering state"):
                statename = data.data[15:] # Cuts the first 16 characters of the message

                # Checks if statename as already occured, if not append to current_states
                if not statename in self.current_states:

                    self.current_states.append(statename)
                    self.start_times.append(tme) # dnb message does not have a timestamp

                else:
                    rospy.loginfo("inconsistent log info, clearing log and starting new")
                    del self.start_times[:]
                    del self.current_states[:]
                    self.current_states.append(statename)
                    self.start_times.append(tme)

                #print len(self.current_states), self.current_states

            else: #Leaving state
                # Find State in State list and remove it
                statename = data.data[14:]
                if statename in self.current_states:
                    idx = self.current_states.index(statename)
                    del self.start_times[idx]
                    self.current_states.remove(statename)


    ###################################
    ## Listener Node
    ###################################
    def listener(self):
        rospy.init_node('forcelogger', anonymous=True)

        # Always start the executor subscriber, so we actually always have to current state
        rospy.Subscriber("/dnb_executor/log", String, self.callback_log)

        rospy.Service('/forcelogger/abortForceLog', Trigger, self.handle_clear)
        rospy.Service('/forcelogger/startForceLog', startForceLog, self.start_forcelog)
        rospy.Service('/forcelogger/stopForceLog', Trigger, self.stop_forcelog)

        # spin() simply keeps python from exiting until this node is stopped
        try:
            rospy.spin()
        finally:
            print("Emergency Saving")
            self.data_file_name += "_emr"
            self.saveLog()
            rospy.logwarn("Emergency save data")

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Logs forces of robots')
    parser.add_argument('-f','--filename', type=str, help='filename for logfile')
    parser.add_argument('-d','--directory', type=str, help='directory')
    parser.add_argument('-b','--breakSize', type=str, help='size when to start emergency saving in MB')

    args = parser.parse_args()
    a = Forcelogger(filename=args.filename,directory=args.directory,breaksize=args.breakSize)
    a.listener()