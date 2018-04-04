#!/usr/bin/env python
import logging
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from pitasc_library.msg import Measurements
from smach_msgs.msg import SmachContainerStatus

import re
import os
import datetime as dt

def decode_filename(filename):
    name = filename
    # Find timestamp and replace according to formating
    for matchObj in re.finditer("{timestamp(.*?)}" ,name):
        #print "1:", matchObj.group()
        if matchObj.group(1) != "": # found custom formating
            #print "2:", matchObj.group(1)
            # Replace timestamp according to input parameters
            name = name.replace(matchObj.group(), "{}".format(dt.datetime.now().strftime(matchObj.group(1))))
        else:
            # Replace timestamp according to predefined (general) parameters
            name = name.replace(matchObj.group(),
                                "{}".format(
                                    dt.datetime.now().strftime(
                                        "%Y-%m-%d_%H:%M:%S")))

    # Change ~ to user
    name = os.path.expanduser(name)

    # Check if folder exists, otherwise create it
    path = os.path.dirname(name)
    if path != '':
        if not os.path.isdir(path):
            rospy.loginfo("Creating new folder: {}".format(path))
            os.makedirs(path)

    rospy.logdebug("Decode Name: {} -> {}".format(filename, name))
    return name

def create_logger(filename, headline):
    logger = logging.getLogger(filename)
    logger.setLevel(logging.INFO)

    # To be replaces with an filelogger with good formating
    #Console stream handler
    loghandler = logging.FileHandler(filename)
    loghandler.setLevel(logging.DEBUG)
    loghandler.setFormatter(logging.Formatter('%(message)s'))
    logger.addHandler(loghandler)
    logger.info("eventtime,{}".format(headline))
    loghandler.setFormatter(logging.Formatter('%(asctime)s,%(message)s',
                                              '%Y-%m-%d %H:%M:%S.%f'))
    return logger

def js_callback(js):
    global _js_log
    _js_log.info("{},{},{},{}".format(js.header.stamp,
                                                ','.join(js.name),
                                                ','.join(map(str, js.position)),
                                                ','.join(map(str, js.velocity))))

def wrench_callback(data):
    global _w_log
    _w_log.info("{},{},{},{},{},{},{}".format(
                data.header.stamp,
                data.wrench.force.x,
                data.wrench.force.y,
                data.wrench.force.z,
                data.wrench.torque.x,
                data.wrench.torque.y,
                data.wrench.torque.z))

def eigenforce3d_callback(data):
    global _eig3_log
    _eig3_log.info("{},{}".format(
                    rospy.Time.now(),
                    data.values[0]))

def eigenforce6d_callback(data):
    global _eig6_log
    _eig6_log.info("{},{}".format(
                    rospy.Time.now(),
                    data.values[0]))

def task_vel_callback(data):
    global _taskvel_log
    _taskvel_log.info("{},{}".format(
                        rospy.Time.now(),
                        ','.join(map(str, data.values[0:6]))))

def Jq_callback(data):
    global _Jq_log
    _Jq_log.info("{},{}".format(
                rospy.Time.now(),
                ','.join(map(str, data.values[0:36]))))

dict_status_num = {}
cnt = 0

def smach_status_callback(data):
    global _status_log, dict_status_num, cnt
    # Exclude Heartbeat messages
    if data.info != "HEARTBEAT":
        statename = '+'.join(map(str, data.active_states))
        if not statename in dict_status_num:
            dict_status_num[statename] = cnt
            cnt += 1

        _status_log.info("{},{},{}".format(
                        data.header.stamp,
                        statename,
                        dict_status_num[statename]))

def control_output_callback(data):
    global _control_output_log
    _control_output_log.info("{},[{}],[{}]".format(
                            rospy.Time.now(),
                            ','.join(map(str, data.symbols)),
                            ','.join(map(str, data.values))))

def control_meas_callback(data):
    global _control_meas_log
    _control_meas_log.info("{},[{}],[{}]".format(
                           rospy.Time.now(),
                           ', '.join(map(str,data.symbols)),
                           ', '.join(map(str, data.values))))

def logger():
    global _js_log,\
           _w_log,\
           _eig6_log,\
           _eig3_log,\
           _taskvel_log,\
           _status_log,\
           _Jq_log,\
           _control_meas_log,\
           _control_output_log

    rospy.init_node('topic_logger')

    prefix = rospy.get_param("~file_prefix", "{timestamp}_")
    prefix = decode_filename(prefix)

    _js_log      = create_logger(prefix + "joint_states.log", 
                                "msg_stamp," + 
                                "q_name[0],q_name[1],q_name[2],q_name[3],q_name[4],q_name[5],"+
                                "q[0],q[1],q[2],q[3],q[4],q[5],"+
                                "q_dot[0],q_dot[1],q_dot[2],q_dot[3],q_dot[4],q_dot[5]")

    _w_log       = create_logger(prefix + "wrench.log",
                                "msg_stamp," + 
                                "force.x,force.y,force.z,torque.x,torque.y,torque.z")

    _eig3_log    = create_logger(prefix + "eigenforce3d.log",
                                 "rostime.now,eigenforce3d")

    _eig6_log    = create_logger(prefix + "eigenforce6d.log",
                                 "rostime.now,eigenforce6d")

    _taskvel_log = create_logger(prefix + "task_vel.log",
                                 "rostime.now,"+
                                 "v_dot[0],v_dot[1],v_dot[2],v_dot[3],v_dot[4],v_dot[5]")

    _Jq_log      = create_logger(prefix + "Jq.log", 
                                 "rostime.now," +
                                 "Jq00,Jq01,Jq02,Jq03,Jq04,Jq05,"+
                                 "Jq10,Jq11,Jq12,Jq13,Jq14,Jq15,"+
                                 "Jq20,Jq21,Jq22,Jq23,Jq24,Jq25,"+
                                 "Jq30,Jq31,Jq32,Jq33,Jq34,Jq35,"+
                                 "Jq40,Jq41,Jq42,Jq43,Jq44,Jq45,"+
                                 "Jq50,Jq51,Jq52,Jq53,Jq54,Jq55")

    _status_log  = create_logger(prefix + "status.log", 
                                 "msg_stamp,"+
                                 "skill_name,"+
                                 "id_number")

    _control_meas_log = create_logger(prefix + "control_measurements.log", 
                                 "rostime.now,"+
                                 "[symbols],"+
                                 "[values]")

    _control_output_log = create_logger(prefix + "control_output.log", 
                                 "rostime.now,"+
                                 "[symbols],"+
                                 "[values]")

    rospy.Subscriber("joint_states", JointState, js_callback)
    rospy.Subscriber("wrench", WrenchStamped, wrench_callback)
    rospy.Subscriber("eigenforce3d", Measurements, eigenforce3d_callback)
    rospy.Subscriber("eigenforce6d", Measurements, eigenforce6d_callback)
    rospy.Subscriber("task_vel", Measurements, task_vel_callback)
    rospy.Subscriber("Jq", Measurements, Jq_callback)
    rospy.Subscriber("pitasc_introspection/smach/container_status", SmachContainerStatus, smach_status_callback)

    rospy.Subscriber("control_measurements", Measurements, control_meas_callback)
    rospy.Subscriber("control_output", Measurements, control_output_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    logger()
