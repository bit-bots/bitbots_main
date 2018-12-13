import rospy
from bitbots_splines.polynom import Polynom, pos, vel, acc, jerk

class Point:
    def __init__(self, time, position, velocity, acceleration):
        self.time = time
        self.position = position
        self.velocity = velocity
        self.acceleration = acceleration

class SplinePart:
    def __init__(self, polynom, min_t, max_t):
        self.polynom = polynom
        self.min = min_t
        self.max = max_t

class SmoothSpline:

    def __init__(self):
        self.points = []
        self.splines_parts = []

    def add_point(self, time,  position,  velocity=0,  acceleration=0):
        self.points.append(Point(time, position, velocity, acceleration))

    def get_points(self):
        return self.points

    def compute_spline(self):
        self.splines_parts = []
        if len(self.points) < 2:
            return
        
        def time(p1):
            return p1.time

        # sort list by time of points
        self.points.sort(key=time)           

        for i in range(0, len(self.points)):
            time = self.points[i].time - self.points[i-1].time
            if time > 0.00001:
                self.splines_parts.append(SplinePart(
                    self.polynom_fit(time,
                        self.points[i-1].position, self.points[i-1].velocity, self.points[i-1].acceleration,
                        self.points[i].position, self.points[i].velocity, self.points[i].acceleration),
                    self.points[i-1].time,
                    self.points[i].time)
                )

    def polynom_fit(self, t, pos1,  vel1,  acc1, pos2,  vel2,  acc2):
        """
        Fit a polynom between 0 and t with given
        pos, vel and acc initial and final conditions
        """
        if t <= 0.00001:
            rospy.logerr("SmoothSpline invalid spline interval")
            exit
        
        t2 = t*t
        t3 = t2*t
        t4 = t3*t
        t5 = t4*t
        p = Polynom()
        p.coefs.append(pos1)
        p.coefs.append(vel1)
        p.coefs.append(acc1/2)
        p.coefs.append(-(-acc2*t2+3*acc1*t2+8*vel2*t+12*vel1*t-20*pos2+20*pos1)/(2*t3))
        p.coefs.append( (-2*acc2*t2+3*acc1*t2+14*vel2*t+16*vel1*t-30*pos2+30*pos1)/(2*t4))
        p.coefs.append(-(-acc2*t2+acc1*t2+6*vel2*t+6*vel1*t-12*pos2+12*pos1)/(2*t5))

        return p


    def interpolation(self, x, func):
        #Empty case
        if len(self.splines_parts) == 0:            
            exit("Error: no spline parts. did you call compute_spline()?")

        #Bound asked abscisse into spline range
        if x <= self.splines_parts[0].min:
            x = self.splines_parts[0].min  

        if x >= self.splines_parts[-1].max:
            x = self.splines_parts[-1].max

        #Bijection spline search
        indexLow = 0
        indexUp = len(self.splines_parts) - 1
        while indexLow != indexUp:
            index = int((indexUp+indexLow)/2)
            if x < self.splines_parts[index].min:
                indexUp = index-1
            elif x > self.splines_parts[index].max:
                indexLow = index+1
            else:
                indexUp = index
                indexLow = index

        #Compute and return spline value
        return func(x-self.splines_parts[indexUp].min, self.splines_parts[indexUp].polynom.coefs)        


    def pos(self, t):
        return self.interpolation(t, pos)

    def vel(self, t):
        return self.interpolation(t, vel)

    def acc(self, t):
        return self.interpolation(t, acc)

    def jerk(self, t):
        return self.interpolation(t, jerk)

