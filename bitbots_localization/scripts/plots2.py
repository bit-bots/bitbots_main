#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import sys
name = '/home/judith/Dokumente/uni/MA/thesis/data/experiment_standing_1/'+sys.argv[1]+'/'+sys.argv[1]
csv_list = []

csv_list = [name + '_1.csv', name + '_2.csv', name + '_3.csv',name +'_4.csv',name + '_5.csv',name +  '_6.csv',name +  '_7.csv',name +  '_7.csv',name +  '_9.csv', name + '_10.csv']

li = []

for file in csv_list:
    df = pd.read_csv(file, index_col=None, header=0)
    li.append(df)

#all 10

if False:
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1)
    fig.figsize=(6.4,10)

    for df in li:
        df.plot(kind='line', x='step', y='dx10', ax=ax1, sharex=True, grid=True,legend=False, figsize=(6.4, 10), ylim=(-8, 8), xlim=(0, 1550)) #linewidth=2
        df.plot(kind='line', x='step', y='dy10', ax=ax2,sharex=True,grid=True,legend=False,ylim=(-8, 8),xlim=(0, 1550))
        df.plot(kind='line', x='step', y='dt10', ax=ax3,sharex=True,grid=True,legend=False,ylim=(-3.5, 3.5),xlim=(0, 1550))
        ax1.set_ylabel('x deviation')
        ax2.set_ylabel('y deviation')
        ax3.set_ylabel('t deviation')
    plt.savefig(sys.argv[1]+'single.png')

#mean

df = pd.read_csv(sys.argv[1]+'_mean.csv', index_col=None, header=0)

fig, (ax1, ax2) = plt.subplots(2, 1)
fig.figsize=(6.4,6.4)

ax1.set_ylabel('x deviation')
df.plot(kind='line', x='step', y='mean euc', ax=ax1, sharex=True, grid=True,legend=False, figsize=(6.4, 6.4), ylim=(0, 8), xlim=(0, 1550))
df.plot(kind='line', x='step', y='min euc', ax=ax1, sharex=True, grid=True,legend=False, figsize=(6.4, 6.4), ylim=(0, 8), xlim=(0, 1550), color=plt.cm.tab10(0))
df.plot(kind='line', x='step', y='max euc', ax=ax1, sharex=True, grid=True,legend=False, figsize=(6.4, 6.4), ylim=(0, 8), xlim=(0, 1550), color=plt.cm.tab10(0))
ax1.fill_between(df['step'], df['min euc'],df['max euc'], alpha=0.3)

#ax2.set_ylabel('y deviation')
#df.plot(kind='line', x='step', y='meany', ax=ax2, sharex=True,grid=True,legend=False,ylim=(-8, 8),xlim=(0, 1550))
#df.plot(kind='line', x='step', y='min y', ax=ax2, sharex=True, grid=True,legend=False, figsize=(6.4, 10), ylim=(0, 8), xlim=(0, 1550), color=plt.cm.tab10(0))
#df.plot(kind='line', x='step', y='max y', ax=ax2, sharex=True, grid=True,legend=False, figsize=(6.4, 10), ylim=(0, 8), xlim=(0, 1550), color=plt.cm.tab10(0))
#ax2.fill_between(df['step'], df['min y'],df['max y'], alpha=0.3)

ax2.set_ylabel('t deviation')
#ax3 = plt.subplot(111, polar=True)
#ax3 = plt.subplot(3, 1, 3, projection='polar')
#df.plot(kind='line', y='step', x='meant', ax=ax3,sharex=True,grid=True,legend=False,xlim=(-3.5, 3.5),ylim=(0, 1550))
df.plot(kind='line', x='step', y='meant', ax=ax2, sharex=True, grid=True, legend=False, ylim=(0, 3.5),xlim=(0, 1550))
df.plot(kind='line', x='step', y='min t', ax=ax2, sharex=True, grid=True, legend=False, figsize=(6.4, 6.4), ylim=(0, 3.5), xlim=(0, 1550), color=plt.cm.tab10(0))
df.plot(kind='line', x='step', y='max t', ax=ax2, sharex=True, grid=True, legend=False, figsize=(6.4, 6.4), ylim=(0, 3.5), xlim=(0, 1550), color=plt.cm.tab10(0))
ax2.fill_between(df['step'], df['min t'], df['max t'], alpha=0.3)

plt.savefig(sys.argv[1]+'_mean_euc.png')

