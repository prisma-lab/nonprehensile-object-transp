#include "kdl_ros_control/kdl_control.h"

KDLController::KDLController(KDLRobot &_robot, KDLObject &_obj)
{
    robot_ = &_robot;
    obj_ = &_obj;
}

Eigen::VectorXd KDLController::sharedCntr(KDL::Frame &_desObjPos,
                                          KDL::Twist &_desObjVel,
                                          KDL::Twist &_desObjAcc,
                                          double &Kp,
                                          double &Kd)
{
    // std::cout << "_desObjVelRot: " << _desObjVel.rot << std::endl << "_desObjVel: " << _desObjVel.vel << std::endl;
    // std::cout << "_desObjAccRot: " << _desObjAcc.rot << std::endl << "_desObjAcc: " << _desObjAcc.vel << std::endl;

    // compute errors in inertial frame
    Eigen::Matrix<double, 6, 1> e, edot;
    computeErrors(_desObjPos, obj_->getFrame(), _desObjVel, obj_->getSpatialVelocity(), e, edot); // make KDL control method
    // std::cout << "e: " << e.transpose() << std::endl << "edot: " << edot.transpose() << std::endl;

    // compute object wrench in body frame
    KDL::Wrench F_b = obj_->computeID();

    double cost;
    // e.block(3,0,3,1) = tiltingTorque(cost, 2, 1e-2);

    std::cout << "F_b ID: " << toEigen(F_b).transpose() << std::endl;
    Eigen::Matrix<double, 6, 1> y = obj_->getMassMatrix()*(spatialRotation(obj_->getFrame().M.Inverse())*toEigen(_desObjAcc) +
                spatialRotation(obj_->getFrame().M.Inverse())*(Kp*e) +
                spatialRotation(obj_->getFrame().M.Inverse())*(Kd*edot));

    F_b = F_b + toKDLWrench(y);
    std::cout << "F_b ID + y: " << toEigen(F_b).transpose() << std::endl;

    obj_->computeContactForces(F_b);
    KDL::Twist Vbdot = obj_->computeFD(F_b);
    std::cout << "Vbdot: " << toEigen(Vbdot).transpose() << std::endl;

    std::vector<KDL::Wrench> w = obj_->getContactsWrenches();
    for (unsigned int i = 0; i < w.size(); i++)
        w.at(i).ReverseSign();
    robot_->setContactsWrenches(w);

    // compute u
    Eigen::VectorXd u;
    u.resize(robot_->getNrJnts());
    KDL::Jacobian Jb(robot_->getObjBodyJac());
    KDL::Jacobian Jbdot(robot_->getObjBodyJacDot()); //
    u = weightedPseudoInverse(robot_->getJsim(), Jb.data)*(toEigen(Vbdot/*_desObjAcc*/)
//        + Kp*spatialRotation(obj_->getFrame().M.Inverse())*e
//        + Kd*spatialRotation(obj_->getFrame().M.Inverse())*edot
        - Jbdot.data*robot_->getJntVelocities())
        - (Eigen::Matrix<double,7,7>::Identity() - weightedPseudoInverse(robot_->getJsim(), Jb.data)*Jb.data)*robot_->getJntVelocities();
    // std::cout << "Jb.data: " << Jb.data << std::endl;
    // std::cout << "u: " << u.transpose() << std::endl;
       std::cout << "getJntVelocities: " << robot_->getJntVelocities().transpose() << std::endl;
    // compute tau
    std::vector<Eigen::Matrix<double,3,7>> Jci = robot_->getContactsBodyJac();
    Eigen::Matrix<double,12,7> Jc = toEigen(Jci);
    Eigen::Matrix<double,12,1> Fc;
    for (unsigned int i = 0; i < robot_->getContacts().size(); i++)
        Fc.block(i*3,0,3,1) = toEigen(robot_->getContacts().at(i).getWrench().force);

    return robot_->getJsim() * u /*- Jc.transpose()*Fc*/ + Jb.data.transpose()*toEigen(F_b)
            + robot_->getCoriolis() + robot_->getGravity();
}

Eigen::VectorXd KDLController::computeFcAngles()
{
    Eigen::VectorXd A;
    A.resize(3*obj_->getContacts().size());

    for(unsigned int i = 0; i < obj_->getContacts().size(); i++)
    {
        Eigen::Vector3d z_hat(0,0,1);
        Eigen::Vector3d Fci = toEigen(obj_->getContacts().at(i).getWrench().force);
        std::cout << "Fci: " << Fci.transpose() << std::endl;
        double normFci = Fci.norm();
        double alpha = std::acos(z_hat.dot(Fci)/normFci);
        Eigen::Vector3d n_hat = Fci.cross(z_hat);
        n_hat = n_hat/n_hat.norm();
        A.block(i*3,0,3,1) = alpha*n_hat;
    }
    return A;
}

Eigen::VectorXd  KDLController::gradientFcAngles(const Eigen::VectorXd &a, const double &lambda, double &costValue)
{
    double h = 0;
    Eigen::VectorXd dh_da;
    dh_da.resize(3*obj_->getContacts().size());
    double theta = obj_->getContacts().at(0).getTheta();
    double t2 = theta*theta;
    for (unsigned int i = 0; i< a.size(); i++)
    {
        h += ((theta*theta)*-4.0)/(lambda*(a(i)+theta)*(a(i)-theta));
        dh_da(i,0) = (a(i)*t2*1.0/pow(t2-a(i)*a(i),2.0)*8.0)/lambda;
        //dh_da(i,0) = -std::pow(2*theta, 2)*2*a(i)/(lambda*((a(i)+theta)*(a(i)-theta)));
    }
    return dh_da;
}

Eigen::VectorXd KDLController::tiltingTorque(double &cost, const double &lambda, const double &K)
{
    Eigen::VectorXd A = computeFcAngles();
    std::cout << "ANGLES: " << A.transpose() << std::endl;
    return -K*obj_->getGraspMatrixOri()*gradientFcAngles(A, lambda, cost);
}

Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd)
{
    // read current state
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // calculate errors
    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;

    Eigen::VectorXd ddqd = _ddqd.data;
    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e)
            + robot_->getCoriolis() + robot_->getGravity() /*friction compensation?*/;
}

Eigen::VectorXd KDLController::idCntr(KDL::Frame &_desPos,
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp, double _Kpo,
                                      double _Kdp, double _Kdo)
{
//    // calculate gain matrices
//    Eigen::Matrix<double,6,6> Kp, Kd;
//    Kp.block(0,0,3,3) = _Kpp*Eigen::Matrix3d::Identity();
//    Kp.block(3,3,3,3) = _Kpo*Eigen::Matrix3d::Identity();
//    Kd.block(0,0,3,3) = _Kdp*Eigen::Matrix3d::Identity();
//    Kd.block(3,3,3,3) = _Kdo*Eigen::Matrix3d::Identity();

//    // read current state
//    Eigen::Matrix<double,6,7> J = robot_->getJacobian();
//    Eigen::Matrix<double,7,7> I = Eigen::Matrix<double,7,7>::Identity();
//    Eigen::Matrix<double,7,7> M = robot_->getJsim();
//    Eigen::Matrix<double,7,6> Jpinv = weightedPseudoInverse(M,J);
//    //Eigen::Matrix<double,7,6> Jpinv = pseudoinverse(J);

//    // position
//    Eigen::Vector3d p_d(_desPos.p.data);
//    Eigen::Vector3d p_e(robot_->getCartesianPose().p.data);
//    Eigen::Matrix<double,3,3,Eigen::RowMajor> R_d(_desPos.M.data);
//    Eigen::Matrix<double,3,3,Eigen::RowMajor> R_e(robot_->getCartesianPose().M.data);
//    R_d = matrixOrthonormalization(R_d);
//    R_e = matrixOrthonormalization(R_e);

//    // velocity
//    Eigen::Vector3d dot_p_d(_desVel.vel.data);
//    Eigen::Vector3d dot_p_e(robot_->getCartesianVelocity().vel.data);
//    Eigen::Vector3d omega_d(_desVel.rot.data);
//    Eigen::Vector3d omega_e(robot_->getCartesianVelocity().rot.data);

//    // acceleration
//    Eigen::Matrix<double,6,1> dot_dot_x_d;
//    Eigen::Matrix<double,3,1> dot_dot_p_d(_desAcc.vel.data);
//    Eigen::Matrix<double,3,1> dot_dot_r_d(_desAcc.rot.data);

//    // compute linear errors
//    Eigen::Matrix<double,3,1> e_p = computeLinearError(p_d,p_e);
//    Eigen::Matrix<double,3,1> dot_e_p = computeLinearError(dot_p_d,dot_p_e);

//    // shared control
//    // Eigen::Vector3d lin_acc;
//    // lin_acc << _desAcc.vel.x(), _desAcc.vel.y(), _desAcc.vel.z(); //use desired acceleration
//    // lin_acc << dot_dot_p_d + _Kdp*(dot_e_p) + _Kpp*(e_p); // assuming no friction no loads
//    // Eigen::Matrix<double,3,3> R_sh = shCntr(lin_acc);

//    // compute orientation errors
//    Eigen::Matrix<double,3,1> e_o = computeOrientationError(R_d,R_e);
//    Eigen::Matrix<double,3,1> dot_e_o = computeOrientationVelocityError(omega_d,
//                                                                        omega_e,
//                                                                        R_d,
//                                                                        R_e);
//    Eigen::Matrix<double,6,1> x_tilde;
//    Eigen::Matrix<double,6,1> dot_x_tilde;
//    x_tilde << e_p, e_o;
//    dot_x_tilde << dot_e_p, -omega_e;//dot_e_o;
//    dot_dot_x_d << dot_dot_p_d, dot_dot_r_d;

//    // null space control
//    double cost;
//    Eigen::VectorXd grad = gradientJointLimits(robot_->getJntValues(),robot_->getJntLimits(),cost);

////    std::cout << "---------------------" << std::endl;
////    std::cout << "p_d: " << std::endl << p_d << std::endl;
////    std::cout << "p_e: " << std::endl << p_e << std::endl;
////    std::cout << "dot_p_d: " << std::endl << dot_p_d << std::endl;
////    std::cout << "dot_p_e: " << std::endl << dot_p_e << std::endl;
////    std::cout << "R_sh*R_d: " << std::endl << R_d << std::endl;
////    std::cout << "R_e: " << std::endl << R_e << std::endl;
////    std::cout << "omega_d: " << std::endl << omega_d << std::endl;
////    std::cout << "omega_e: " << std::endl << omega_e << std::endl;
////    std::cout << "x_tilde: " << std::endl << x_tilde << std::endl;
////    std::cout << "dot_x_tilde: " << std::endl << dot_x_tilde << std::endl;
////    std::cout << "jacobian: " << std::endl << robot_->getJacobian() << std::endl;
////    std::cout << "jpinv: " << std::endl << Jpinv << std::endl;
////    std::cout << "jsim: " << std::endl << robot_->getJsim() << std::endl;
////    std::cout << "c: " << std::endl << robot_->getCoriolis().transpose() << std::endl;
////    std::cout << "g: " << std::endl << robot_->getGravity().transpose() << std::endl;
////    std::cout << "q: " << std::endl << robot_->getJntValues().transpose() << std::endl;
////    std::cout << "Jac Dot qDot: " << std::endl << robot_->getJacDotqDot().transpose() << std::endl;
////    std::cout << "Jnt lmt cost: " << std::endl << cost << std::endl;
////    std::cout << "Jnt lmt gradient: " << std::endl << grad.transpose() << std::endl;
////    std::cout << "---------------------" << std::endl;

//    // inverse dynamics
//    Eigen::Matrix<double,6,1> y;
//    y << dot_dot_x_d - robot_->getJacDotqDot() + Kd*dot_x_tilde + Kp*x_tilde;

//    return M * (Jpinv*y + (I-Jpinv*J)*(/*- 10*grad */- 1*robot_->getJntVelocities()))
//            + robot_->getGravity() + robot_->getCoriolis();
}

