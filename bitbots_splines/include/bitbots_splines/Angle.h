/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef ANGLE_H
#define ANGLE_H

#include <cmath>

namespace bitbots_splines {

/**
 * Return the given angle in radian 
 * bounded between -PI and PI
 */
inline double AngleBound(double angle)
{
    return 
        angle 
        - 2.0*M_PI*std::floor((angle + M_PI)/(2.0*M_PI));
}

/**
 * Compute the oriented distance between the two given angle
 * in the range -PI/2:PI/2 radian from angleSrc to angleDst
 * (Better than doing angleDst-angleSrc)
 */
inline double AngleDistance(double angleSrc, double angleDst) 
{    
    angleSrc = AngleBound(angleSrc);
    angleDst = AngleBound(angleDst);

    double max, min;
    if (angleSrc > angleDst) {
        max = angleSrc;
        min = angleDst;
    } else {
        max = angleDst;
        min = angleSrc;
    }

    double dist1 = max-min;
    double dist2 = 2.0*M_PI - max + min;
 
    if (dist1 < dist2) {
        if (angleSrc > angleDst) {
            return -dist1;
        } else {
            return dist1;
        }
    } else {
        if (angleSrc > angleDst) {
            return dist2;
        } else {
            return -dist2;
        }
    }
}

/**
 * Compute a weighted average between the 
 * two given angles in radian.
 * Returned  angle is between -PI and PI.
 */
inline double AngleWeightedMean(
    double weight1, double angle1, double weight2, double angle2)
{
    double x1 = cos(angle1);
    double y1 = sin(angle1);
    double x2 = cos(angle2);
    double y2 = sin(angle2);

    double meanX = weight1*x1 + weight2*x2;
    double meanY = weight1*y1 + weight2*y2;

    return atan2(meanY, meanX);
}
        
}

#endif

