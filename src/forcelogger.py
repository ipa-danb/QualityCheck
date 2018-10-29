#!/usr/bin/env python

# ROS imports
import rospy
from geometry_msgs.msg import WrenchStamped
from robot_movement_interface.msg import EulerFrame
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger
from quality_check.srv import startForceLog
from dnb_msgs.msg import ComponentStatus

# Regular imports
import os
import sys
import threading
import time
import pandas as pd
import numpy as np
import argparse

class Forcelogger:

    def __init__(self,**kwargs):
        try:
            if kwargs['filename'] is None:
                self.data_file_name = "data"
            else:
                self.data_file_name = kwargs['filename']
        except KeyError:
            self.data_file_name = "data"

        rospy.loginfo("Using filename ", self.data_file_name)

        try:
            self.breaksize = int(kwargs['breaksize'])
        except:
            self.breaksize = 20
        rospy.loginfo("Using maximum cache of ", self.breaksize, " MB")


        try:
            self.minspace = int(kwargs['minspace'])
        except:
            self.minspace = 10*1024 # Default 10 GB space

        rospy.loginfo("Requireing minimum free space of ", self.minspace, " MB")

        try:
            path = os.path.expanduser('~')
            if os.path.isdir(kwargs['directory']) and kwargs['directory'] is not None:
                self.directory = os.path.join(path,kwargs['directory'])
            else:
                self.directory = os.path.join(path,".dnb","data")
        except:
            self.directory = os.path.join(path,".dnb","data")

        try:
            os.makedirs(self.directory)
        except OSError:
            if not os.path.isdir(self.directory):
                raise

        rospy.loginfo("Using path: ", self.directory)

        # Initialize logging lists
        self.timings = []
        self.list_training_data = [[],[]]
        self.current_states = []
        self.start_times = []
        self.last_measurement = {'x_position': 0,
                                 'y_position': 0,
                                 'z_position': 0,
                                 'alpha_position': 0,
                                 'beta_position': 0,
                                 'gamma_position': 0,
                                 'time_position':0}
        self.start_time = 0
        self.counter = 0
        self.runcounter = 0

        # Initialize Lock
        self.lock = threading.Lock()

    ###################################
    ## Handles for service callbacks
    ###################################

    def start_forcelog(self,req):
        self.runcounter += 1
        if self.runcounter > 1:
            rospy.loginfo("Forcelog already running")
            return False
        else:
            rospy.loginfo("Start Forcelog with filename: " + req.name)
            self.data_file_name = req.name
            if self.memorycheck():
                self.clearLog()
                self.publishstatus(2,'Force log is running')
                self.subscribe()
                return True
            else:
                self.publishstatus(2,'Not enough memory left!')
                return False

    def save_and_clear(self):
        self.saveLog()
        self.clearLog()

    def stop_forcelog(self,req):
        if runcounter > 1:
            self.runcounter -= 1
            return False
        else:
            try:
                self.unsubscribe()
                threading.Thread(target = save_and_clear, args = (self,)).start()
                rospy.loginfo("Stopping Forcelog")
                rospy.loginfo("---")
                if self.memorycheck():
                    self.publishstatus(2,'Force log is paused')
                else:
                    self.publishstatus(2,'Not enough memory left!')


                return True, ''
            except Exception as ex:
                return False, str(ex)

    def handle_clear(self,req):
        """Clears datalog
        """
        try:
            self.unsubscribe()
            self.clearLog()
            return True, 'Dicarded Recording'
        except Exception as ex:
            return False, str(ex)

    ###################################
    ## Support fcns
    ###################################

    def saveLog(self):
        # Create File Name with timestring
        timestr = time.strftime("%Y%m%d-%H%M%S")
        foldstr = time.strftime("%Y%m%d")
        file_name = '{0}_{1}'.format(self.data_file_name,timestr)

        dir = os.path.join(self.directory,foldstr)

        start_time = time.time()

        if not self.list_training_data:
            rospy.loginfo("No Data to save")
            return False
        try:
            # Parquet with Brotli compression
            try:
                os.makedirs(dir)
            except OSError:
                if not os.path.isdir(dir):
                    raise

            length = len(sorted(self.list_training_data[1],key=len, reverse=True)[0])
            preind = [tuple(xi+['-']*(length-len(xi))) for xi in self.list_training_data[1]]
            mltind = pd.MultiIndex.from_tuples(preind)
            df2 = pd.DataFrame(self.list_training_data[0],index=mltind)
            df2.to_parquet(os.path.join(dir,file_name), compression='brotli')

            rospy.loginfo("Saved file as %s", file_name)
        except Exception as ex:
            rospy.logerr(ex)

        end_time = time.time()

        rospy.loginfo("Time needed for saving: {0:.2f}s".format(end_time- start_time))

    def clearLog(self):
        self.list_training_data = [[],[]]
        self.last_measurement = {'x_position': 0,
                                 'y_position': 0,
                                 'z_position': 0,
                                 'alpha_position': 0,
                                 'beta_position': 0,
                                 'gamma_position': 0,
                                 'time_position':0}
        rospy.loginfo("Cleared datalog")

    def memorycheck(self):
        # check for free memory in data directory: OS specific! (unix only)
        st = os.statvfs(self.directory)
        mb = st.f_bavail * st.f_frsize / 1024 / 1024
        rospy.loginfo("so minspace {0} and mb {1}".format(self.minspace,mb) )
        return self.minspace < mb

    def publishstatus(self,status,message):
        #TODO: this is probably redundant
        self.cm_status = ComponentStatus()
        if status == 0:
            self.cm_status.status_id = ComponentStatus().INITIALIZED
        elif status == 1:
            self.cm_status.status_id = ComponentStatus().STOPPED
        elif status == 2:
            self.cm_status.status_id = ComponentStatus().RUNNING
        elif status == 3:
            self.cm_status.status_id = ComponentStatus().CONFIG_NEEDED
        else:
            self.cm_status.status_id = ComponentStatus().ERROR
        self.cm_status.status_msg = message
        self.status_publisher.publish(self.cm_status)

    ###################################
    ## Subscribe & Unsubscribe functions
    ###################################

    def subscribe(self):
        rospy.loginfo("Subscribing to /tool_frame  and /wrench")
        self.sub_tf_chain = rospy.Subscriber("/tool_frame",EulerFrame,self.callback_tfchain)
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
        with self.lock:
            tme_pos =  rospy.Time.now()
            self.last_measurement = {'x_position': data.x,
                                     'y_position': data.y,
                                     'z_position': data.z,
                                     'alpha_position': data.alpha,
                                     'beta_position': data.beta,
                                     'gamma_position': data.gamma,
                                     'time_position':tme_pos.to_sec()}


    # This function is called whenever a wrench data set arrives
    def callback_wrench(self,data):

        with self.lock:

            self.counter += 1
            tme_force = rospy.Time(secs=data.header.stamp.secs,nsecs=data.header.stamp.nsecs)
            for state in self.current_states:
                idx = self.current_states.index(state)
                self.start_time = self.start_times[idx]


            meas = {'x_force': data.wrench.force.x,
                    'y_force': data.wrench.force.y,
                    'z_force': data.wrench.force.z,
                    'x_torque': data.wrench.torque.x,
                    'y_torque': data.wrench.torque.y,
                    'z_torque': data.wrench.torque.z,
                    'time_force': tme_force.to_sec(),
                    'time_start': self.start_time.to_sec()}

            meas.update(self.last_measurement)

            try:
                self.list_training_data[0].append(meas)
                self.list_training_data[1].append(list(self.current_states))
            except:
                print("Topic doesnt exist")

            if self.counter % 10000 == 0:
                self.counter = 0
                if sys.getsizeof(self.list_training_data)/(1024.0*1024.0) > self.breaksize:
                    rospy.logwarn("Size of Log over {0} MB, save data, stopping Logger".format(self.breaksize))
                    self.unsubscribe()
                    self.saveLog()
                    self.clearLog()

    # This function is called whenever a information about the active state arrives
    def callback_log(self,data):
        with self.lock:
            tme_log = rospy.Time.now()
            # Checks if a new state is entered
            if data.data.startswith("Entering state"):
                statename = data.data[15:] # Cuts the first 16 characters of the message

                # Checks if statename as already occured, if not append to current_states
                if not statename in self.current_states:

                    self.current_states.append(statename)
                    self.start_times.append(tme_log) # dnb message does not have a timestamp

                else:
                    rospy.loginfo("inconsistent log info, clearing log and starting new")
                    del self.start_times[:]
                    del self.current_states[:]
                    self.current_states.append(statename)
                    self.start_times.append(tme_log)

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

        # topic for d&b componentmanager
        self.status_publisher = rospy.Publisher('/forcelogger/status', ComponentStatus, queue_size=5, latch=True)

        # Always start the executor subscriber, so we actually always have to current state
        rospy.Subscriber("/dnb_executor/log", String, self.callback_log)

        rospy.Service('/forcelogger/abortForceLog', Trigger, self.handle_clear)
        rospy.Service('/forcelogger/startForceLog', startForceLog, self.start_forcelog)
        rospy.Service('/forcelogger/stopForceLog', Trigger, self.stop_forcelog)

        if self.memorycheck():
            self.publishstatus(0,'ready for recording')
        else:
            self.publishstatus(2,'Not enough memory left!')

        # spin() simply keeps python from exiting until this node is stopped
        try:
            rospy.spin()
        finally:
            self.publishstatus(2,'Emergency Stop')
            print("Emergency Saving")
            self.data_file_name += "_emr"
            self.saveLog()
            rospy.logwarn("Emergency save data")

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Logs forces of robots')
    parser.add_argument('-f', '--filename', type=str, help='filename for logfile')
    parser.add_argument('-d', '--directory', type=str, help='directory')
    parser.add_argument('-b', '--breakSize', type=int, help='size when to start emergency saving in MB')
    parser.add_argument('-m', '--minspace', type=int, help='stop saving data if not at least this much space is left in MB')

    args = parser.parse_args(rospy.myargv()[1:]) # from ROS there could be arguments which interfere with argparse
    a = Forcelogger(filename=args.filename,directory=args.directory,breaksize=args.breakSize)
    a.listener()
