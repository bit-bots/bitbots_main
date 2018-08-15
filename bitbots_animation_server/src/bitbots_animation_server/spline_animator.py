import rospy
from bitbots_animation_server.animation import Keyframe, Animation
from bitbots_splines.smooth_spline import SmoothSpline

class SplineAnimator:

    def __init__(self, animation, current_joint_states=None):
        self.anim = animation
        self.start_time = rospy.get_time()
        self.animation_duration = 0
        self.current_point_time = 0
        self.spline_dict = {}        

        #TODO add current joint positions as start

        #load keyframe positions into the splines
        for keyframe in self.anim.keyframes:            
            self.animation_duration += keyframe.duration + keyframe.pause
            self.current_point_time += keyframe.duration
            for joint in keyframe.goals:
                if joint not in self.spline_dict:
                    self.spline_dict[joint] = SmoothSpline()
                self.spline_dict[joint].add_point(self.current_point_time, keyframe.goals[joint])
                self.spline_dict[joint].add_point(self.current_point_time + keyframe.pause, keyframe.goals[joint])                
            self.current_point_time += keyframe.pause       

        # compute the splines
        for joint in self.spline_dict:
            self.spline_dict[joint].compute_spline()
         
    
    def get_positions(self, time):
        if time < 0 or time > self.animation_duration:
            return None
        ret_dict = {}
        for joint in self.spline_dict:
            ret_dict[joint] = self.spline_dict[joint].pos(time)
        return ret_dict
    
    def get_start_time(self):
        return self.start_time
        
    def get_duration(self):
        return self.animation_duration