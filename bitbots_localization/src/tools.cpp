#include <bitbots_localization/tools.h>

std::pair<double,double> cartesianToPolar(double x, double y)
{
    double r = hypot(x, y);

    double t = atan2(y, x);

    return std::make_pair(t, r);


     }

std::pair<double, double> polarToCartesian(double t, double r){
    double x = r * std::cos(t);
    double y = r * std::sin(t);

    return std::make_pair(x, y);
}


