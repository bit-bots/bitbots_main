#ifndef COM_CALCULATOR_HPP
#define COM_CALCULATOR_HPP

#include "kinematic_robot.hpp"
#include "../debug/debug.hpp"
#include <Eigen/Eigen>
#include <vector>
#include "boost/date_time/posix_time/posix_time.hpp"

namespace Robot {
namespace Kinematics {

namespace pa = Debug::Paint;

static const Eigen::Vector4f ORIGIN(0,0,0,1);
static const Eigen::Vector4f Y_AXIS(0,1,0,0);
static const Eigen::Vector4f X_AXIS(0,0,1,0);//In meinen Sinne x, nicht z

//Note the foot size here
const static Eigen::Vector2f l_foot_ul_offset(0.052, 0.044);//korrigieren
const static Eigen::Vector2f l_foot_ur_offset(0.052, -0.021);//Füße
const static Eigen::Vector2f l_foot_bl_offset(-0.052, 0.044);//sind
const static Eigen::Vector2f l_foot_br_offset(-0.052, -0.021);//nur
const static Eigen::Vector2f r_foot_ul_offset(0.052, 0.021);//66
const static Eigen::Vector2f r_foot_ur_offset(0.052, -0.044);//mm
const static Eigen::Vector2f r_foot_bl_offset(-0.052, 0.021);//breit
const static Eigen::Vector2f r_foot_br_offset(-0.052, -0.044);

class Center_Of_Mass_Calculator{

private:
KRobot robot;
mutable Debug::Scope debug;
std::vector<pa::Shape> debug_shapes;
char last_leg;
Eigen::Vector4f last_com, movement;
float mass;
boost::posix_time::ptime time;
boost::posix_time::time_duration delta_time;
Eigen::Vector3f accelleration, speed;

Eigen::Vector2f l_foot_ul;
Eigen::Vector2f l_foot_ur;
Eigen::Vector2f l_foot_bl;
Eigen::Vector2f l_foot_br;
Eigen::Vector2f r_foot_ul;
Eigen::Vector2f r_foot_ur;
Eigen::Vector2f r_foot_bl;
Eigen::Vector2f r_foot_br;



const float leg_uncertenty;

enum {RIGHT = -1, NONE = 0, LEFT = 1};

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Center_Of_Mass_Calculator()
    :debug("ZMP-Project"), last_leg(NONE), leg_uncertenty(0.003),
    last_com(0,0,0,0), movement(0,0,0,0)
    {
        mass = robot.get_mass();
        time = boost::posix_time::microsec_clock::local_time();
    }

    /**
     * Reset the Tracking of the leg, for ex when fallen down
     */
    void lost_leg();

    /**
     * Update the calculation
     */
    void update(Pose& pose);
    void print_chains(Pose& pose);

    float get_pulse()
    {
        return movement.norm() * mass * (delta_time.total_milliseconds() / 1000.0);
    }

    /**
     * Returns the robot's movement.
     */
    Eigen::Vector3f get_movement()
    {
        return movement.head<3>();
    }

    Eigen::Vector3f get_accelleration()
    {
        return accelleration;
    }

    float get_mass()
    {
        return mass;
    }

private:

    /**
     * projects the feet to ground and then calculates the polygon discribed
     * by the robots feet.
     * At least decides, wheather the masspoint is in or outside the poligon.
     */
    void project_feet(Eigen::Vector3f l, Eigen::Vector3f r, Eigen::Vector2f com);

    /**
     * Calculates the robot's midpoint, for the two legs and decides on which
     * it's standing on.
     */
    Eigen::Matrix4f get_robot_midpoint();

    /**
     * Tracks the position of the center of mass in relation to the feet.
     * Thus this method tracks the movement of the center of mass.
     */
    void track_masscenter_movement(const Eigen::Matrix4f& l_foot_inv
        , const Eigen::Matrix4f& r_foot_inv);

};

} } //namespace

#endif
