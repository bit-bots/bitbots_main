If you get an import error from the following line:
    from humanoid_league_speaker.cfg import speaker_paramsConfig
Do from catkin directory:
    cp src/humanoid_league_misc/humanoid_league_speaker/cfg/speaker_paramsConfig.cfg devel/lib/python2.7/dist-packages/humanoid_league_speaker/cfg
And rebuild.

I don't know why this problem exists.
