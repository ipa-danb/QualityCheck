#!/usr/bin/env python
import rospy
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import String
import pandas as pd
from std_srvs.srv import Empty
import matplotlib.pyplot as plt
import os
import threading
import pickle
import datetime
import time

class Forcelogger:
    def __init__(self,**kwargs):
        try:
            if kwargs['filename'] is None:
                self.data_file_name = "data"
            else:
                self.data_file_name = kwargs['filename']
        except KeyError:
            self.data_file_name = "data"

        self.dict_training_data = dict()
        self.dict_training_data = {}
        self.current_states = []
        self.start_times = []
        self.lock = threading.Lock()

    def handle_show(self,req):
        for state in self.dict_training_data:
            print state
            #print dict_training_data[state]
            try:
                ax = pd.DataFrame(self.dict_training_data[state][0]).plot()
                if len(self.dict_training_data[state]) > 1:
                    for i in range(1, len(self.dict_training_data[state])):
                        pd.DataFrame(self.dict_training_data[state][i]).plot(ax=ax)
                plt.show()
            except:
                pass
        return []

    def handle_save(self,req):
        timestr = time.strftime("%Y%m%d-%H%M%S")
        file_name = '{0}_{1}'.format(timestr,self.data_file_name)
        with open( file_name, "wb" ) as f:
        	pickle.dump(self.dict_training_data, f)
        	rospy.loginfo("Saved file as %s", file_name)
    	self.dict_training_data.clear()
    	rospy.loginfo("... and cleared datalog")
        return []

    """
    Is that even useful?

        def handle_load(req):
            global dict_training_data, current_states, start_times, data_file_name

            # clean up running acqusition
            self.dict_training_data = pickle.load(open( data_file_name, "rb" ))
            self.current_states = []
            self.start_times = []

            return []
    """

    # This function is called whenever a wrench data set arrives (now does nothing)
    def callback_wrench(self,data):
        tme =  rospy.Time.now().to_sec() #TODO: Use timestamp
        with self.lock:
            for state in self.current_states:
                idx = self.current_states.index(state)
                start_time = self.start_times[idx]

                #print "---->", tme, start_time, tme - start_time

                meas = {'x': data.wrench.force.x,
                        'y': data.wrench.force.y,
                        'z': data.wrench.force.z,
                        'time': tme - start_time}

                self.dict_training_data[state][-1].append(meas)

    # This function is called whenever a information about the active state arrives
    def callback_log(self,data):
        tme = rospy.Time.now().to_sec()
        with self.lock:
            # Checks if a new state is entered
            if data.data.startswith("Entering state"):
                statename = data.data[15:] # Cuts the first 16 characters of the message

                #print "<--enter", statename, tme
                # Checks if statename as already occured, if not append to current_states
                if not statename in self.current_states:

                    self.current_states.append(statename)
                    self.start_times.append(tme) # dnb message does not have a timestamp
                    if statename in self.dict_training_data:
                        self.dict_training_data[statename].append([])
                    else:
                        # add state
                        self.dict_training_data[statename] = [[]]
                else:
                    print "State appeared twice???????? PROBLEM????"

                print len(self.current_states), self.current_states

            else: #Leaving state
                statename = data.data[14:]
                if statename in self.current_states:
                    idx = self.current_states.index(statename)
                    del self.start_times[idx]
                    self.current_states.remove(statename)

                    #print "exit-->", statename, tme

    def listener(self):

        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('listener', anonymous=True)

        rospy.Subscriber("/wrench", WrenchStamped, self.callback_wrench)
        rospy.Subscriber("/dnb_executor/log", String, self.callback_log)

        #rospy.Service('show', Empty, self.handle_show)
        rospy.Service('save', Empty, self.handle_save)
        #rospy.Service('load', Empty, self.handle_load)

        # spin() simply keeps python from exiting until this node is stopped
        try:
            rospy.spin()
        finally:
            print "Emergency Saving"
            self.data_file_name += "_emr"
            self.handle_save(None)
            print "ER save data in the following states (still running)\n", self.current_states

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Logs forces of robots')
    parser.add_argument('-f','--filename', type=str, help='filename for logfile')

    args = parser.parse_args()
    print args.filename
    a = Forcelogger(filename=args.filename)
    a.listener()
