//
// Created by judith on 09.03.19.
//

#include "../include/bitbots_localization/RobotState.h"


RobotState::RobotState() :
        m_XPos(0.0),
        m_YPos(0.0),
        m_SinTheta(0.0),
        m_CosTheta(1.0),
        is_explorer_(false)
{

}

RobotState::RobotState(double x, double y, double T) :
        m_XPos(x),
        m_YPos(y),
        m_SinTheta(sin(T)),
        m_CosTheta(cos(T)),
        is_explorer_(false)
{

}

RobotState::~RobotState()
{
}

RobotState RobotState::operator*(float factor) const
{
    RobotState newState;
    newState.m_XPos = m_XPos * factor;
    newState.m_YPos = m_YPos * factor;
    newState.m_SinTheta = m_SinTheta * factor;
    newState.m_CosTheta = m_CosTheta * factor;
    return newState;
}

RobotState& RobotState::operator+=(const RobotState& other)
{
    m_XPos += other.m_XPos;
    m_YPos += other.m_YPos;
    m_SinTheta += other.m_SinTheta;
    m_CosTheta += other.m_CosTheta;
    return *this;
}


double RobotState::getXPos() const
{
    return m_XPos;
}

double RobotState::getYPos() const
{
    return m_YPos;
}

double RobotState::getTheta() const
{
    return atan2(m_SinTheta, m_CosTheta);
}

double RobotState::getSinTheta() const
{
    return m_SinTheta;
}

double RobotState::getCosTheta() const
{
    return m_CosTheta;
}

void RobotState::setXPos(double x)
{
    m_XPos = x;
}

void RobotState::setYPos(double y)
{
    m_YPos = y;
}


void RobotState::setTheta(double t)
{
    m_SinTheta = sin(t);
    m_CosTheta = cos(t);
}

void RobotState::setSinTheta(double st)
{
    m_SinTheta = st;
}

void RobotState::setCosTheta(double ct){
    m_SinTheta = ct;
}


double RobotState::calcDistance(const RobotState& state) const {
    double diff =  std::sqrt(std::pow(getXPos() - state.getXPos(), 2) + std::pow(getYPos() - state.getYPos(), 2));
    if (diff == 0.0) {
        diff = 0.0001;
    }
    return diff;
}

void RobotState::convertParticleListToEigen(const std::vector<particle_filter::Particle<RobotState> *> &particle_list,
                                            Eigen::MatrixXd &matrix,
                                            const bool ignore_explorers) {
    if (ignore_explorers) {
        int non_explorer_count = 0;
        for (particle_filter::Particle<RobotState> *particle :
                particle_list) {
            if (!particle->is_explorer_) {
                non_explorer_count++;
            }
        }

        matrix.resize(non_explorer_count, 3);
        int counter = 0;
//#pragma parallel for
        for (particle_filter::Particle<RobotState> *particle :
                particle_list) {
            if (!particle->is_explorer_) {
                matrix(counter, 0) = particle->getState().getXPos();
                matrix(counter, 1) = particle->getState().getYPos();
                matrix(counter, 2) = particle->getState().getTheta();

                counter++;
            }
        }
    } else {
        matrix.resize(particle_list.size(), 3);
//#pragma parallel for
        for (int i = 0; i < particle_list.size(); i++) {
            matrix(i, 0) = particle_list[i]->getState().getXPos();
            matrix(i, 1) = particle_list[i]->getState().getYPos();
            matrix(i, 2) = particle_list[i]->getState().getTheta();

        }
    }
}

visualization_msgs::Marker RobotState::renderMarker(std::string n_space, std::string frame, ros::Duration lifetime, std_msgs::ColorRGBA color) const {
    visualization_msgs::Marker msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame;
    msg.action = visualization_msgs::Marker::ADD;
    msg.type = visualization_msgs::Marker::ARROW;
    tf2::Quaternion q;
    q.setEuler(0, 0, getTheta());
    q.normalize();
    msg.pose.orientation.w = q.w();
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.position.x = getXPos();
    msg.pose.position.y = getYPos();
    msg.scale.x = 0.3;
    msg.scale.y = 0.04;
    msg.scale.z = 0.04;
    msg.color.r = color.r;
    msg.color.g = color.g;
    msg.color.a  = 1;
    msg.lifetime = lifetime;
    msg.ns = n_space;
    msg.id = reinterpret_cast<uint64_t>(this);

    return msg;
}