#!/usr/bin/env python
import rospy
from pitasc_library.msg import Measurements
from smach_msgs.msg import SmachContainerStatus
import matplotlib.pyplot as plt

_start = False
_eig3_log = []

def plot():
    global _eig3_log
    fig, ax = plt.subplots()
    ax.plot(range(len(_eig3_log)), _eig3_log)
    plt.show(block=False)
    _eig3_log = []


def eigenforce3d_callback(data):
    global _eig3_log, _start
    if _start:
        print data
        _eig3_log.append(data.values[0])

def smach_status_callback(data):
    global _start
    # Exclude Heartbeat messages
    if data.info != "HEARTBEAT":
        print data.active_states
        if not _start and data.active_states[0] == "denso_move_ptp":
            _start = True
            print "Starting"
        elif _start and not data.active_states[0] == "denso_move_ptp":
            _start = False
            print "Stopping"
            plot()

if __name__ == '__main__':

    rospy.init_node('eig_plotter')

    rospy.Subscriber("pitasc_introspection/smach/container_status", SmachContainerStatus, smach_status_callback)
    rospy.Subscriber("eigenforce3d", Measurements, eigenforce3d_callback)
 
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
