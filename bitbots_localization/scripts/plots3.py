#!/usr/bin/env python3
import os

list = ['all', 'fb', 'fb_features', 'features', 'goals', 'goals_fb', 'goals_fb_features', 'goals_features', 'lines',
        'lines_fb', 'lines_fb_features', 'lines_features', 'lines_goals', 'lines_goals_fb', 'lines_goals_features']

for i in list:
    #os.system("python3 ~/robocup/bitbots_meta/humanoid_league_localization/scripts/plots.py " + i)
    #os.system("python3 ~/robocup/bitbots_meta/humanoid_league_localization/scripts/plots2.py " + i)
    os.system("python3 ~/robocup/bitbots_meta/humanoid_league_localization/scripts/abs.py " + i)
