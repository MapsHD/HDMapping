#!/usr/bin/env python
# coding: utf-8

import matplotlib.pyplot as plt
import numpy as np
import sys

if len(sys.argv)!=3:
    print("Usage:\n%s hist.csv func.csv"%sys.argv[0])
    exit(1)
hist_fname = sys.argv[1] #'hist.csv'
func_fname = sys.argv[2] #'func.csv'

def read_csv_data(fname):
    with open(fname) as f:
        data = f.read().split('\n')
        data = data[1:] # skip header
        data = [_.split(',') for _ in data]
        if len(data[-1])!=2:
            data = data[:-1]
        data = list(zip(*[(float(_[0]),float(_[1])) for _ in data]))
#         data = [data[0],np.array(data[1])/np.max(data[1])]
        return data

func_data = read_csv_data(func_fname)
hist_data = read_csv_data(hist_fname)


hist_name = ('.'.join(hist_fname.split('.')[:-1])).replace('\\','/').split('/')[-1]
func_name = ('.'.join(func_fname.split('.')[:-1])).replace('\\','/').split('/')[-1]

plt.bar(*hist_data, width=0.75*(hist_data[0][1]-hist_data[0][0]),label=hist_name)
plt.plot(*func_data,'r',label=func_name)
plt.legend() # comment this to disable legend

plt.savefig(hist_name + '-' + func_name + '.png')
plt.show()


