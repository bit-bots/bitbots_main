#include  "InverseKinematics.h"
#include "NUBots/message/motion/KinematicsModels.h"
#include "ForwardKinematics.h"

struct Coordinates
{
double x;
double y;
double z;
double alpha;
double beta;
double gamma;
};

typedef message::motion::kinematics::KinematicsModel DarwinModel;

std::vector<float> berechneIK(Coordinates Left, Coordinates Right, Coordinates Oberkoerper)
{
  utility::math::matrix::Transform3D torso = arma::vec6({Oberkoerper.x, Oberkoerper.y, Oberkoerper.z, Oberkoerper.alpha, Oberkoerper.beta, Oberkoerper.gamma});

  utility::math::matrix::Transform3D leftFoot = arma::vec6({Left.x, Left.y, Left.z, Left.alpha, Left.beta, Left.gamma});
  utility::math::matrix::Transform3D rightFoot = arma::vec6({Right.x, Right.y, Right.z, Right.alpha, Right.beta, Right.gamma});

  utility::math::matrix::Transform3D leftFootTorso = leftFoot.worldToLocal(torso);
  utility::math::matrix::Transform3D rightFootTorso = rightFoot.worldToLocal(torso);

  std::vector<std::pair<message::input::ServoID, float>> joints = utility::motion::kinematics::calculateLegJointsTeamDarwin(DarwinModel(), leftFootTorso, rightFootTorso);

  std::vector<float> tmp(30,0.0);

for (std::pair<message::input::ServoID, float> &motor : joints)
{
tmp.at(motor.first) = motor.second;
}
return tmp;
}


Coordinates calculateLegDirektKinematics(const std::vector<float>& positionen, bool links)
{
DarwinModel dModel{};

message::input::Sensors sensoren{};
sensoren.servos = std::vector<message::input::Sensors::Servo>{30,message::input::Sensors::Servo{}};
for (int i = 0; i < 30; i ++)
{
sensoren.servos[i].presentPosition = positionen[i];
}

std::map<message::input::ServoID, utility::math::matrix::Transform3D> positions =  utility::motion::kinematics::calculateLegJointPosition(dModel, sensoren, links?message::input::L_ANKLE_ROLL:message::input::R_ANKLE_ROLL, links?message::motion::kinematics::LEFT:message::motion::kinematics::RIGHT);

auto pos = positions[links?message::input::L_ANKLE_ROLL:message::input::R_ANKLE_ROLL];

auto trans = pos.translation();
auto rot = pos.eulerAngles();

Coordinates coord{};
coord.x = trans[0];
coord.y = trans[1];
coord.z = trans[2];
coord.alpha = rot[0];
coord.beta = rot[1];
coord.gamma = rot[2];
return coord;
}