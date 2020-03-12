#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys

list = ('lines',  'goals', 'fb', 'features', 'lines_goals',  'lines_fb', 'lines_features', 'goals_fb', 'goals_features', 'fb_features',  'goals_fb_features',
       'lines_fb_features', 'lines_goals_features', 'lines_goals_fb', 'all')
meaneuc = []
meant = []
x = np.arange(len(list))
for i in list:
    file = '/home/judith/Dokumente/uni/MA/thesis/images/experiment/standing_3/'+ i +'_mean.csv'
    df = pd.read_csv(file, index_col=None, header=0)
    meaneuc.append(df['mean euc'].iloc[500:].mean(axis=0))
    meant.append(df["meant"].iloc[500:].mean(axis=0))

fig, (ax1, ax2) = plt.subplots(2,1, sharex=True)
fig.figsize=(10,6.4)

ax1.bar(x, meaneuc)
ax1.set_ylim([0,4])
ax2.bar(x, meant)
ax2.set_ylim([0,1.25])

plt.xticks(x, list, rotation='vertical')
plt.savefig('bar_means.png')
