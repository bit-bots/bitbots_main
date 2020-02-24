#!/usr/bin/env python3
import os
import pandas as pd
import glob
import csv
import math
import sys

csv_header = ['step', 'meanx', 'meany', 'mean euc','meant', 'min x', 'max x', 'min y', 'max y','min euc', 'max euc', 'min t', 'max t']
csv_out = sys.argv[1]+'_mean.csv'
name = '/home/judith/Dokumente/uni/MA/thesis/data/experiment_standing_1/'+sys.argv[1]+'/'+sys.argv[1]

csv_list = []

csv_list = [name + '_1.csv', name + '_2.csv', name + '_3.csv',name +'_4.csv',name + '_5.csv',name +  '_6.csv',name +  '_7.csv',name +  '_7.csv',name +  '_9.csv', name + '_10.csv']
li = []

for file in csv_list:
    df = pd.read_csv(file, index_col=None, header=0)
    li.append(df)

lenlist = []

for df in li:
    lenlist.append(len(df.index))

with open(csv_out, 'w') as f:
    csv_writer = csv.writer(f)
    csv_writer.writerow(csv_header)  # write header

for line in range(0, min(lenlist)):
    sumx = 0
    sumy = 0
    sumt = 0
    sumSin = 0
    sumCos = 0
    x = []
    y = []
    t = []
    euc = []
    with open(csv_out, 'a+') as f:
        csv_writer = csv.writer(f)
        for df in li:
            x.append(abs(df.iat[line, 48]))
            y.append(abs(df.iat[line, 49]))
            euc.append(math.hypot(abs(df.iat[line, 48]), abs(df.iat[line, 49])))
            sumSin += math.sin(abs(df.iat[line, 50]))
            sumCos += math.cos(abs(df.iat[line, 50]))
            t.append(abs(df.iat[line, 50]))
        meanx = sum(x) / 10
        meany = sum(y) /10
        mean_euc = sum(euc) / 10
        meanSumSin = sumSin / 10
        meanSumCos = sumCos / 10
        meant = math.atan2(meanSumSin, meanSumCos)
        row = [line, meanx, meany, mean_euc, meant, min(x), max(x), min(y), max(y), min(euc), max(euc), min(t), max(t)]
        csv_writer.writerow(row)



