#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys

list = ('lines',  'goals', 'fb', 'features', 'lines_goals',  'lines_fb', 'lines_features', 'goals_fb', 'goals_features', 'fb_features',  'goals_fb_features',
       'lines_fb_features', 'lines_goals_features', 'lines_goals_fb', 'all')
meaneuc = []
meant = []

df_euc = pd.DataFrame()
df_ang = pd.DataFrame()


for i in list:
    file1 = '/home/judith/Dokumente/uni/MA/thesis/data/experiment_standing_1/'+ i +'_abs.csv'
    file2 = '/home/judith/Dokumente/uni/MA/thesis/data/experiment_standing_2/'+ i +'_abs.csv'
    file3 = '/home/judith/Dokumente/uni/MA/thesis/data/experiment_standing_3/'+ i +'_abs.csv'

    df1 = pd.read_csv(file1, index_col=None, header=0)
    df2 = pd.read_csv(file2, index_col=None, header=0)
    df3 = pd.read_csv(file3, index_col=None, header=0)
    df = df1.append(df2.append(df3,ignore_index=True),ignore_index=True)
    #df = df3

    df_euc[i] = df['abs_euc']
    df_ang[i] = df['abs_t']

fig, (ax1, ax2) = plt.subplots(2,1, sharex=True)
fig.figsize=(10,20)

df_euc.boxplot(ax=ax1, showfliers=False)
df_ang.boxplot(ax=ax2, showfliers=False)

plt.xticks(rotation='vertical')
plt.savefig('boxplot_standing.png')
