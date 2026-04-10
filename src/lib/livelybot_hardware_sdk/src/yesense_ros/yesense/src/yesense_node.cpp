#include "yesense_driver.h"

int main(int argc, char** argv)
{
    ros::init(argc,argv,"yesense_imu");
    ros::NodeHandle nh, nh_private("~");

    yesense::YesenseDriver yesense_dirver(nh,nh_private);
    yesense_dirver.run();

    return 0;
}