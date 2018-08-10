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
import matplotlib.pyplot as plt
import datetime
import argparse
parser = argparse.ArgumentParser(description='Logs forces of robots')
parser.add_argument('-f','--filename', type=str, help='filename for logfile', default='SpeedTest500mmx20')
parser.add_argument('-a','--analyze', type=str, help='analyze dnb subprogram', default='Insert Lever')
parser.add_argument('-n','--number', type=int, help='number for analysis', default=1)


args = parser.parse_args()

fname = args.filename
fr = args.analyze
nrAnalysis = args.number

def findstuff(index,stuff):
    for i,el in enumerate(index.levels):
        for el2 in el:
            if stuff in el2:
                return True,el2,i

    return False,''

def extractStats(fileName,selector):
    df = pq.read_table(fileName).to_pandas()
    aw = findstuff(df.index,selector)
    df2 = df.reset_index()
    ar = df2.loc[df2['level_'+str(aw[2])] == aw[1]][['xf','yf','zf']]
    return list(ar.mean()), list(ar.std()),list(ar.max()-ar.min())

datadir = '/home/dragandbot/data'
#fname = 'LeverZ2InputTest'
#fr = 'Force Log Context'
#fr = 'Insert_'
#nrAnalysis = 2

plt.ion()
plt.figure(figsize=(30,16))
ax1 = plt.subplot(211)
ax2 = plt.subplot(212)



while True:
    dataNameList = glob.glob(datadir +'/*')

    #print(dataNameList)
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

    rm = [(os.path.basename(f)[0:-16],os.path.basename(f)[-15:-1],f) for f in dataNameList_f]
    choosen = [k for k in rm if k[0] == fname]
    choosen.sort(key= lambda x: datetime.datetime.strptime(x[1],"%Y%m%d-%H%M%S"))


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

    plt.pause(2)
