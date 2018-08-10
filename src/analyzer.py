#!/usr/bin/env python
import rospy
from geometry_msgs.msg import WrenchStamped
from robot_movement_interface.msg import EulerFrame
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger
from quality_check.srv import startForceLog, analyzeForceData, analyzeForceDataResponse


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
import glob
from sklearn.covariance import MinCovDet
import datetime
import argparse



datadir = '/home/dragandbot/data'
cfgFolder = "cfgs"

#fname = 'LeverZ2InputTest'
#fr = 'Force Log Context'
#fr = 'Insert_'
#nrAnalysis = 2
"""
plt.ion()
plt.figure(figsize=(30,16))
ax1 = plt.subplot(211)
ax2 = plt.subplot(212)



while True:
    dataNameList = glob.glob(datadir +'/*')

    # load cfg file
    try:
        with open(fname + fr + '.cfg','rb') as f:
            cfg = pickle.load(f)
            dataNameList_f = list(set(dataNameList) - set(cfg['dataNameList']))
            ab = cfg['statlist']
            mcd = cfg['mcd']
        print('loaded config')
    except:
        print('no configFile')
        dataNameList_f = dataNameList
        ab = list()
        mcd = MinCovDet()

    # load new file names
    rm = [(os.path.basename(f)[0:-16],os.path.basename(f)[-15:-1],f) for f in dataNameList_f]
    choosen = [k for k in rm if k[0] == fname]
    choosen.sort(key= lambda x: datetime.datetime.strptime(x[1],"%Y%m%d-%H%M%S"))


    # extract all stats
    for k1 in choosen:
        a1,a2,a3 = extractStats(k1[2],fr)
        a1.extend(a2)
        a1.extend(a3)
        ab.append(a1)

    rr = np.array(ab)

    #print(dataNameList_f)
    if dataNameList_f:
        print('fitting '+ str(len(dataNameList_f)) + ' new data')
        mcd.fit(rr[:-1*nrAnalysis-1,:])
    else:
        print('no new data')


    arn = mcd.mahalanobis(rr[-1*nrAnalysis-1:-1,:]- mcd.location_)** (0.33)
    aro = mcd.mahalanobis(rr[:-1*nrAnalysis-1,:]- mcd.location_)** (0.33)

    print(np.median(aro[mcd.support_]) )


    ax1.clear()
    ax1.scatter(rr[:-1*nrAnalysis-1,[0]],rr[:-1*nrAnalysis-1,[3]],marker='+')
    ax1.scatter(rr[-1*nrAnalysis-1:-1,[0]],rr[-1*nrAnalysis-1:-1,[3]],marker='o',c='r')

    #ax1.scatter(*mcd.location_,c='r',s=4)

    ax2.clear()
    datrange= np.arange(0,len(aro))
    ax2.plot(datrange,aro,c='b')
    ax2.scatter(datrange[mcd.support_], aro[mcd.support_],c='g',marker='o')
    ax2.scatter(np.arange(len(aro),len(aro)+len(arn)), arn,c='r')
    ax2.axhline(y=np.median(aro[mcd.support_]), color='r', linestyle='-')
    ax2.axhline(y=np.median(aro[mcd.support_])+3*np.std(aro[mcd.support_]),color='r',linestyle='--')
    ax2.axhline(y=np.median(aro[mcd.support_])-3*np.std(aro[mcd.support_]),color='r',linestyle='--')


    plt.title(str(np.median(aro[mcd.support_]))+' | '+str(np.mean(arn)) + '_|_' + fr)

    cfg = {}
    cfg['statlist']= ab
    cfg['mcd'] = mcd
    cfg['dataNameList'] = dataNameList
    with open(fname + fr + '.cfg','wb') as f:
        pickle.dump(cfg,f)

    plt.show()

"""

def plotstuff(rr,nrAnalysis,aro,arn,mcd,fr):

    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(30,16))
    ax1 = plt.subplot(211)
    ax2 = plt.subplot(212)

    ax1.clear()
    ax1.scatter(rr[:-1*nrAnalysis-1,[0]],rr[:-1*nrAnalysis-1,[3]],marker='+')
    ax1.scatter(rr[-1*nrAnalysis-1:-1,[0]],rr[-1*nrAnalysis-1:-1,[3]],marker='o',c='r')

    ax2.clear()
    datrange= np.arange(0,len(aro))
    ax2.plot(datrange,aro,c='b')
    ax2.scatter(datrange[mcd.support_], aro[mcd.support_],c='g',marker='o')
    ax2.scatter(np.arange(len(aro),len(aro)+len(arn)), arn,c='r')
    ax2.axhline(y=np.median(aro[mcd.support_]), color='r', linestyle='-')
    ax2.axhline(y=np.median(aro[mcd.support_])+3*np.std(aro[mcd.support_]),color='r',linestyle='--')
    ax2.axhline(y=np.median(aro[mcd.support_])-3*np.std(aro[mcd.support_]),color='r',linestyle='--')

    plt.title(str(np.median(aro[mcd.support_]))+' | '+str(np.mean(arn)) + '_|_' + fr)
    plt.show(block=False)
    return True

class Analyzer(object):
    def __init__(self, **kwargs):
        self.datadir = '/home/dragandbot/data'
        self.cfgFolder = '/home/dragandbot/cfgs'
        self.dataNameList = glob.glob(self.datadir +'/*')

    def saveConfig(self,cfg,cfgName):
        with open(cfgName,'wb') as f:
            pickle.dump(cfg,f)



    def loadCfgFile(self,filename):
        self.dataNameList = glob.glob(self.datadir +'/*')

        try:
            with open(filename,'rb') as f:
                cfg = pickle.load(f)
                dataNameList_f = list(set(self.dataNameList) - set(cfg['dataNameList']))
                ab = cfg['statlist']
                mcd = cfg['mcd']
            rospy.loginfo('loaded config')
        except:
            dataNameList_f = self.dataNameList
            ab = list()
            mcd = MinCovDet()
        return mcd, ab, dataNameList_f

    def findstuff(self,index,stuff):
        for i,el in enumerate(index.levels):
            for el2 in el:
                if stuff in el2:
                    return True,el2,i

        return False,''

    def extractStats(self,fileName,selector):
        df = pq.read_table(fileName).to_pandas()
        aw = self.findstuff(df.index,selector)
        df2 = df.reset_index()
        ar = df2.loc[df2['level_'+str(aw[2])] == aw[1]][['xf','yf','zf']]
        return list(ar.mean()), list(ar.std()),list(ar.max()-ar.min())

    def handle_callback(self,req):

        nrAnalysis = req.nr
        selector = req.selector
        fname = req.fname
        cfgName = os.path.join(self.cfgFolder,fname + selector + '.cfg')

        rospy.loginfo("Querying {0} in {1} with {2} number of stuff".format(fname,selector,nrAnalysis))
        rospy.loginfo(cfgName)

        mcd, ab, dataNameList_f = self.loadCfgFile(cfgName)
        rospy.loginfo("number of updated files: {0}".format(len(dataNameList_f)))
        choosen = self.loadNewFiles(dataNameList_f,fname)

        for element in choosen:
            a1,a2,a3 = self.extractStats(element[2],selector)
            a1.extend(a2)
            a1.extend(a3)
            ab.append(a1)

        rr = np.array(ab)
        #rospy.loginfo(self.dataNameList)
        rospy.loginfo(choosen)
        rospy.loginfo(ab)

        if dataNameList_f:
            rospy.loginfo('fitting {0} new data with format {1}'.format(len(dataNameList_f),rr.shape))
            mcd.fit(rr[:-1*nrAnalysis-1,:])
        else:
            rospy.loginfo('no new data')

        arn = mcd.mahalanobis(rr[-1*nrAnalysis-1:-1,:]- mcd.location_)** (0.33)
        aro = mcd.mahalanobis(rr[:-1*nrAnalysis-1,:]- mcd.location_)** (0.33)

        cfg = {}
        cfg['statlist']= ab
        cfg['mcd'] = mcd
        cfg['dataNameList'] = self.dataNameList

        self.saveConfig(cfg,cfgName)
        #plotstuff(rr,nrAnalysis,aro,arn,mcd,selector)

        """
        bool success
        bool checkTrigger
        float64 qcvalue
        """

        resp = analyzeForceDataResponse()
        resp.success = True
        diff = 3*np.std(aro[mcd.support_[0:len(aro)]]) # 3 times std == 99.95%

        if np.mean(arn) > (np.mean(aro[mcd.support_[0:len(aro)]])+ diff) or np.mean(arn) < (np.mean(aro[mcd.support_[0:len(aro)]]) - diff):
            resp.checkTrigger = True
        else:
            resp.checkTrigger = False


        resp.qcvalue = (np.mean(arn) - np.mean(aro[mcd.support_[0:len(aro)] ]) )/np.std(aro[mcd.support_[0:len(aro)]])

        return resp


    def loadNewFiles(self,dataNameList_f,fname):
        rm = [(os.path.basename(f)[0:-16],os.path.basename(f)[-15:-1],f) for f in dataNameList_f]
        choosen = [k for k in rm if k[0] == fname]
        choosen.sort(key= lambda x: datetime.datetime.strptime(x[1],"%Y%m%d-%H%M%S"))
        return choosen

    def listener(self):
        rospy.init_node('analyzer', anonymous=True)

        # TODO give those meaningful names
        rospy.Service('/analyzer/analyze', analyzeForceData, self.handle_callback)
        #rospy.Service('/forcelogger/stopForceLog', Trigger, self.stop_forcelog)

        # spin() simply keeps python from exiting until this node is stopped
        try:
            rospy.spin()
        finally:
            rospy.logwarn("Shutting down analyzer node")

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Logs forces of robots')
    parser.add_argument('-f','--filename', type=str, help='filename for logfile', default='SpeedTest500mmx20')
    parser.add_argument('-a','--analyze', type=str, help='analyze dnb subprogram', default='Insert Lever')
    parser.add_argument('-n','--number', type=int, help='number for analysis', default=1)


    args = parser.parse_args()

    fname = args.filename
    fr = args.analyze
    nrAnalysis = args.number

    a = Analyzer()
    a.listener()
