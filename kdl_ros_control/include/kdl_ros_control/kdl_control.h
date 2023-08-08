#ifndef KDLControl
#define KDLControl

#include "Eigen/Dense"
#include "kdl_robot.h"
#include "kdl_object.h"
#include "utils.h"

class KDLController
{

public:

    KDLController(KDLRobot &_robot, KDLObject &_obj);

    Eigen::VectorXd idCntr(KDL::JntArray &_qd,
                           KDL::JntArray &_dqd,
                           KDL::JntArray &_ddqd,
                           double _Kp,
                           double _Kd);

    Eigen::VectorXd idCntr(KDL::Frame &_desPos,
                           KDL::Twist &_desVel,
                           KDL::Twist &_desAcc,
                           double _Kpp,
                           double _Kpo,
                           double _Kdp,
                           double _Kdo);

    Eigen::VectorXd sharedCntr(KDL::Frame &_desObjPos,
                           KDL::Twist &_desObjVel,
                           KDL::Twist &_desObjAcc,
                           double Kp,
                           double Kd,
                           bool _useTilting);

private:

    Eigen::VectorXd tiltingTorque(double &cost, const double &lambda, const double &K);
    Eigen::VectorXd gradientFcAngles(const Eigen::VectorXd &a, const double &lambda, double &costValue);
    Eigen::VectorXd computeFcAngles();
    KDLRobot* robot_;
    KDLObject* obj_;

};

#endif
