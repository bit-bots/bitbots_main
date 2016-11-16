#include "com_calculator.hpp"

#include "kinematic_joint.hpp"
#include "../debug/debug.hpp"
#include <Eigen/Eigen>
#include <boost/foreach.hpp>
#include <vector>
#include "boost/date_time/posix_time/posix_time.hpp"

#define foreach BOOST_FOREACH

using Robot::Pose;
using namespace Robot::Kinematics;
using namespace std;
using namespace Eigen;

/**
 * Efficient method to find out wheather the angle, discribed by the
 * three parameter points is in range 0-pi(positive return) or
 * 0-(-pi)(negative return)
 */
inline float ccw_f(float v_0x, float v_0y, float v_1x, float v_1y, float v_2x, float v_2y)
{
    return (v_1x - v_0x) * (v_2y - v_0y) - (v_2x - v_0x) * (v_1y - v_0y);
}

void Center_Of_Mass_Calculator::lost_leg()
{
    last_leg = NONE;
}

static inline void print_chain(const typename ::Robot::Kinematics::KRobot::Chain& chain) {
    foreach(const KJoint& joint, chain){
        cout<<"Next Joint"<<endl<<joint.get_endpoint();
    }
}

void Center_Of_Mass_Calculator::print_chains(Pose& pose) {
    robot.update(pose);
    const typename ::Robot::Kinematics::KRobot::Chain& head = robot.get_chain_by_name("Head"),
                                                       larm = robot.get_chain_by_name("LArm"),
                                                       rarm = robot.get_chain_by_name("RArm"),
                                                       lleg = robot.get_chain_by_name("LLeg"),
                                                       rleg = robot.get_chain_by_name("RLeg");


    cout<< "Head Chain"<<endl;
    print_chain(head);
    cout<< "Head LArm"<<endl;
    print_chain(larm);
    cout<< "Head RArm"<<endl;
    print_chain(rarm);
    cout<< "Head LLeg"<<endl;
    print_chain(lleg);
    cout<< "Head RLeg"<<endl;
    print_chain(rleg);

    Matrix4f c = robot.get_joint_by_name("LFootEndpoint").get_chain_matrix_inverse().cast<float>();
    cout<<"Stuff und so"<<endl<<c<<endl<<c*robot.get_joint_by_name("LElbow").get_chain_matrix().cast<float>()<<endl;
    cout<<"LeftBase"<<endl<<robot.get_joint_by_name("LFootEndpoint").get_chain_matrix_inverse() * robot.get_joint_by_name("RFootEndpoint").get_chain_matrix()<<endl
       <<"RightBase"<<endl<<robot.get_joint_by_name("RFootEndpoint").get_chain_matrix_inverse() * robot.get_joint_by_name("LFootEndpoint").get_chain_matrix()<<endl;
}

void Center_Of_Mass_Calculator::update(Pose& pose) {
    boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
    delta_time = now - time;
    time = now;
    debug_shapes.clear();

    robot.update(pose);

    Matrix4f robot_midpoint = get_robot_midpoint();

    Vector3f l_foot_midpoint = (robot.get_joint_by_name("LFootEndpoint").get_chain_matrix().cast<float>() * robot_midpoint * ORIGIN).head<3>();
    Vector3f r_foot_midpoint = (robot.get_joint_by_name("RFootEndpoint").get_chain_matrix().cast<float>() * robot_midpoint * ORIGIN).head<3>();
    Vector3f com = (robot_midpoint * robot.get_center_of_gravity().cast<float>()).head<3>();
    Vector2f projected_com(com(2), com(0));
    debug_shapes.push_back(pa::circle(projected_com, 0.01, pa::Blue));
    project_feet(l_foot_midpoint, r_foot_midpoint, projected_com);

    debug("Center.X") = com.z();
    debug("Center.Y") = com.y();
    debug("Center.Z") = com.x();
    debug("Masse") = robot.get_mass();
}

void Center_Of_Mass_Calculator::project_feet(Vector3f l, Vector3f r, Vector2f com)
{
    //Discribing the polygon

    Vector2f l_2d(l(2), l(0));
    Vector2f r_2d(r(2), r(0));

    r_foot_ur = r_2d + r_foot_ur_offset;
    r_foot_ul = r_2d + r_foot_ul_offset;
    r_foot_bl = r_2d + r_foot_bl_offset;
    r_foot_br = r_2d + r_foot_br_offset;
    l_foot_ur = l_2d + l_foot_ur_offset;
    l_foot_ul = l_2d + l_foot_ul_offset;
    l_foot_bl = l_2d + l_foot_bl_offset;
    l_foot_br = l_2d + l_foot_br_offset;
    pa::Color color = pa::Green;
    if(last_leg == LEFT)
    {
        color = pa::Pink;
    }
    //Pushing the right foot into debug
    debug_shapes.push_back(pa::line(r_foot_ur, r_foot_ul, color));
    debug_shapes.push_back(pa::line(r_foot_ul, r_foot_bl, color));
    debug_shapes.push_back(pa::line(r_foot_bl, r_foot_br, color));
    debug_shapes.push_back(pa::line(r_foot_br, r_foot_ur, color));
    if(last_leg == LEFT)
    {
        color = pa::Green;
    }
    //Pushing the left foot into debug
    debug_shapes.push_back(pa::line(l_foot_ur, l_foot_ul, color));
    debug_shapes.push_back(pa::line(l_foot_ul, l_foot_bl, color));
    debug_shapes.push_back(pa::line(l_foot_bl, l_foot_br, color));
    debug_shapes.push_back(pa::line(l_foot_br, l_foot_ur, color));
    vector<Vector2f> polygon;
    if(last_leg != RIGHT){
        polygon.push_back(l_foot_ur);
        polygon.push_back(l_foot_ul);
        polygon.push_back(l_foot_bl);
        polygon.push_back(l_foot_br);
    }
    if(last_leg != LEFT){
        polygon.push_back(r_foot_bl);
        polygon.push_back(r_foot_br);
        polygon.push_back(r_foot_ur);
        polygon.push_back(r_foot_ul);
    }

    //die Art für die ccw Methode aus der Vision kopiert und um 1 verschoben
    int m = 2;
    for(int i = 3; i < polygon.size() + 1; ++i)
    {
        while(ccw_f(polygon[m-1].x() , polygon[m-1].y(),
            polygon[m].x(), polygon[m].y(), polygon[i%polygon.size()].x(),
            polygon[i%polygon.size()].y()) <= 0)
        {
            --m;
            if(m == 0) break;
        }
        ++m;
        polygon[m%polygon.size()] = polygon[i%polygon.size()];
    }
    bool in_polygon = true;
    for(int i = 0; i < m; ++i)
    {
        if(ccw_f(polygon[i].x() , polygon[i].y(),
            polygon[(i + 1)%m].x(), polygon[(i + 1)%m].y()
            , com.x(), com.y()) <= 0)
        {
            in_polygon = false;
            debug("FootAbort")=i;
            break;
        }
    }
    //discribing the Polygon in Debug shapes
    for(int i = 0; i < m+1; ++i)
    {
        debug_shapes.push_back(pa::line(polygon[(i%m+1)%polygon.size()], polygon[((i+1)%m+1)%polygon.size()], pa::Cyan));
    }

    if(!in_polygon)
    {
        debug << "Center of mass not in Polygon -,-";
    }
    else
    {
        debug << "In Polygon :)";
    }

    debug("Foot_Shapes") = debug_shapes;
    debug("Foot_Shapes.Draw") = true;
    debug("Foot_Shapes.Number") = m;
    debug("ComX") = com.x();
    debug("ComY") = com.y();

    debug("Shapes.1X")=polygon[0].x();
    debug("Shapes.1Y")=polygon[0].y();
    debug("Shapes.2X")=polygon[1].x();
    debug("Shapes.2Y")=polygon[1].y();
    debug("Shapes.3X")=polygon[2].x();
    debug("Shapes.3Y")=polygon[2].y();
    debug("Shapes.4X")=polygon[3].x();
    debug("Shapes.4Y")=polygon[3].y();
    if(m==6)
    {
        debug("Shapes.5X")=polygon[4].x();
        debug("Shapes.5Y")=polygon[4].y();
        debug("Shapes.6X")=polygon[5].x();
        debug("Shapes.6Y")=polygon[5].y();
    }


}

Matrix4f Center_Of_Mass_Calculator::get_robot_midpoint()
{
    const Matrix4f& l_foot = robot.get_joint_by_name("LFootEndpoint").get_chain_matrix_inverse().cast<float>();
    const Matrix4f& r_foot = robot.get_joint_by_name("RFootEndpoint").get_chain_matrix_inverse().cast<float>();

    const Matrix4f& l_leg = robot.get_joint_by_name("LFootEndpoint").get_chain_matrix().cast<float>();
    const Matrix4f& r_leg = robot.get_joint_by_name("RFootEndpoint").get_chain_matrix().cast<float>();
    float l_to_r = (l_foot * r_leg * ORIGIN).transpose() * Y_AXIS;
    float r_to_l = (r_foot * l_leg * ORIGIN).transpose() * Y_AXIS;

    debug("B l_to_r") = l_to_r;
    debug("B r_to_l") = r_to_l;
    debug("Bein") = "";

    if(l_to_r < 0 && r_to_l > leg_uncertenty)
    {
        //unsure = false;
        last_leg = RIGHT;
        debug("Bein") = "Rechts";
        return r_foot;
    }
    else if(r_to_l < 0 && l_to_r > leg_uncertenty)
    {
        //unsure = false;
        last_leg = LEFT;
        debug("Bein") = "Links";
        return l_foot;
    }
    else
    {
        if(l_to_r< leg_uncertenty && r_to_l < leg_uncertenty)
        {
            last_leg = NONE;
        }
        //Kann man das mit Drehungsmatrizen machen???
        switch(last_leg){
            case(RIGHT):{
                debug("Bein") = "Rechts";
                return r_foot;
            }
            case(LEFT):{
                debug("Bein") = "Links";
                return l_foot;
            }
            default:{
                debug("Bein") = "Beide";
                return (l_foot+r_foot)/2;
            }
        }
    }
}

void Center_Of_Mass_Calculator::track_masscenter_movement(
    const Matrix4f& l_foot_inv, const Matrix4f& r_foot_inv)
{
    Vector4f l_com = l_foot_inv * ORIGIN;
    Vector4f r_com = r_foot_inv * ORIGIN;
    Vector4f& com = l_com;
    float foot_changing_delta = 0;

    switch(last_leg){
        case RIGHT:
        com = r_com;
        break;
        case NONE:
        if((float)(last_com.transpose() * X_AXIS) > 0 && r_com.transpose() * X_AXIS < 0)
        {
            com = r_com;
        }
        if((float)(last_com.transpose() * X_AXIS) > 0)
        {
            foot_changing_delta = abs((float)(r_com.transpose() * X_AXIS) -
                (float)(l_com.transpose() * X_AXIS));
        }
        break;
    }

    Vector4f mov = com - last_com + Vector4f(0,0,foot_changing_delta,0);
    Vector3f sp = (mov - movement).head<3>() / (delta_time.total_milliseconds()/1000.0);
    accelleration = (sp - speed) / (delta_time.total_milliseconds()/1000.0);
    speed = sp;
    movement = mov;
    last_com = com;
}
