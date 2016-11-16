#-*- coding:utf-8 -*-

from bitbots.robot.pypose import PyPose


""" Hier liegen verschiedene Methoden zum Initialisieren von Testdaten
Anfangs geplant für Posen und blaaa"""

def sleep(time, statemachine):
    """ Simulating a waited time for a statemachine
    
    :param time time in seconds wich is simulated as waiting

    :param statemachine statemachine that will be manipulated. """
    if statemachine is not None and time is not None:
        statemachine.next_deadline -= time

def get_init_pose():
    pose = PyPose()
    pose["LAnklePitch"].position = -29.971
    pose["LAnkleRoll"].position = -0.791
    pose["LElbow"].position = -15.0
    pose["LHipPitch"].position = 36.123 
    pose["LHipRoll"].position = -0.352 
    pose["LHipYaw"].position = 0.0 
    pose["LKnee"].position = -53.174 
    pose["LShoulderPitch"].position = 45.0 
    pose["LShoulderRoll"].position = 0.0 
    pose["RAnklePitch"].position = 29.971 
    pose["RAnkleRoll"].position = 0.791 
    pose["RElbow"].position = 15.0 
    pose["RHipPitch"].position = -36.123 
    pose["RHipRoll"].position = 0.352 
    pose["RHipYaw"].position = 0.0
    pose["RKnee"].position = 53.174 
    pose["RShoulderPitch"].position = -45.0 
    pose["RShoulderRoll"].position = 0.0
    return pose



def get_Ipc():
    
    class IpcMock():
        
        def __init__(self):
            # Simulated Constants
            self.CONTROLABLE = 0

            # Real Variable
            self.controlable = 1

        def update(self, pose):
            pose["LAnklePitch"].position = pose["LAnklePitch"].goal
            pose["LAnkleRoll"].position = pose["LAnkleRoll"].goal
            pose["LElbow"].position = pose["LElbow"].goal
            pose["LHipPitch"].position = pose["LHipPitch"].goal
            pose["LHipRoll"].position = pose["LHipRoll"].goal
            pose["LHipYaw"].position = pose["LHipYaw"].goal
            pose["LKnee"].position = pose["LKnee"].goal
            pose["LShoulderPitch"].position = pose["LShoulderPitch"].goal
            pose["LShoulderRoll"].position = pose["LShoulderRoll"].goal
            pose["RAnklePitch"].position = pose["RAnklePitch"].goal 
            pose["RAnkleRoll"].position = pose["RAnkleRoll"].goal
            pose["RElbow"].position = pose["RElbow"].goal
            pose["RHipPitch"].position = pose["RHipPitch"].goal 
            pose["RHipRoll"].position = pose["RHipRoll"].goal 
            pose["RHipYaw"].position = pose["RHipYaw"].goal
            pose["RKnee"].position = pose["RKnee"].goal 
            pose["RShoulderPitch"].position = pose["RShoulderPitch"].goal
            pose["RShoulderRoll"].position = pose["RShoulderRoll"].goal
            pose["HeadTilt"].position = pose["HeadTilt"].goal
            pose["HeadPan"].position = pose["HeadPan"].goal
            pass
            
    return IpcMock()
