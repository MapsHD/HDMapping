#!/usr/bin/env python
# coding: utf-8

from glob import glob
import pandas as pd
import matplotlib.pyplot as plt


dirs = [_ for _ in glob('../data/RESSO/*')]
print(dirs)


def process_figure(d):
    fname=d+'/result_win.csv'
    df0 = pd.read_csv(fname, sep=';')
    ref = df0.iloc[0] # first row = ref
    df0 = df0.iloc[1:] # remove first row
    
    fname=d+'/result_linux.csv'
    df1 = pd.read_csv(fname, sep=';')
    df1 = df1.iloc[1:] # remove first row
    df = pd.concat((df0,df1)) #merge
    return ref,df


for d in dirs:
    print('Processing:',d)

    ref,df = process_figure(d)
    df.plot.scatter('id_method','rms', figsize=(8,4))
    plt.yscale('log')
    plt.plot([ref['rms']]*100,'r',label="Reference")
    plt.xlabel('Method')
    plt.ylabel('RMS')
    plt.legend();
    
    out_fname = d+'/plot.eps'
    plt.savefig(out_fname)
    print("Saved:",out_fname)
    print()
