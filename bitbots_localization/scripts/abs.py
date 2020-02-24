#!/usr/bin/env python3
import pandas as pd
import csv
import math
import sys

csv_header = ['step', 'abs_euc', 'abs_t']
csv_out = sys.argv[1]+'_abs.csv'
name = '/home/judith/Dokumente/uni/MA/thesis/data/experiment_standing_3/'+sys.argv[1]+'/'+sys.argv[1]

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

    with open(csv_out, 'a+') as f:
        csv_writer = csv.writer(f)
        for df in li:
            euc = math.hypot(abs(df.iat[line, 48]), abs(df.iat[line, 49]))
            t = abs(df.iat[line, 50])
            row = [line, euc, t]
            csv_writer.writerow(row)



