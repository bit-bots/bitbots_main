#ifndef BITBOTS_ROS_CONTROL_FOOT_PRESSURE_SENSOR_INTERFACE_H
#define BITBOTS_ROS_CONTROL_FOOT_PRESSURE_SENSOR_HANDLE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <string>


/** \brief A handle used to read the state of a four point foot pressure sensor.
 *
 */
class FootPressureSensorHandle
{
public:

    FootPressureSensorHandle(
        std::string name;                       ///< name of the sensor
        std::string frame_id;                   ///< reference frame to which this sensor is associated
        double left_back;                       ///< pressure value measured on different points of the sensor
        double left_front;
        double right_front;
        double right_back;
    )
            : name_(name),
              frame_id_(frame_id),
              left_back_(left_back),
              left_front_(left_front),
              right_front_(right_front),
              right_back_(right_back)
    {}

    std::string getName()           const {return name_;}
    std::string getFrameId()        const {return frame_id_;}
    const double* getLeftBack()     const {return left_back_;}
    const double* getLeftFront()    const {return left_front_;}
    const double* getRightFront()   const {return right_front_;}
    const double* getRightBack()    const {return right_back_;}

private:
    std::string name_;
    std::string frame_id_;

    const double left_back_;
    const double left_front_;
    const double right_front_;
    const double right_back_;
};

class FootPressureSensorInterface : public HardwareResourceManager<FootPressureSensorHandle> {};

}

#endif //BITBOTS_ROS_CONTROL_FOOT_PRESSURE_SENSOR_INTERFACE_H
