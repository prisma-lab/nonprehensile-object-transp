#include "kdl_ros_control/kdl_robot.h"

KDLRobot::KDLRobot()
{

}

KDLRobot::KDLRobot(KDL::Tree &robot_tree)
{
    createChain(robot_tree);
    n_ = chain_.getNrOfJoints();
    grav_ = KDL::JntArray(n_);
    s_J_ee_ = KDL::Jacobian(n_);
    b_J_ee_ = KDL::Jacobian(n_);
    s_J_dot_ee_ = KDL::Jacobian(n_);
    b_J_dot_ee_ = KDL::Jacobian(n_);
    b_J_obj_ = KDL::Jacobian(n_);
    s_J_obj_ = KDL::Jacobian(n_);
    s_J_dot_obj_= KDL::Jacobian(n_);
    b_J_dot_obj_= KDL::Jacobian(n_);
    s_J_ee_.data.setZero();
    b_J_ee_.data.setZero();
    s_J_dot_ee_.data.setZero();
    b_J_dot_ee_.data.setZero();
    b_J_obj_.data.setZero();
    s_J_obj_.data.setZero();
    s_J_dot_obj_.data.setZero();
    b_J_dot_obj_.data.setZero();
    b_J_c_.resize(4);
    obj_ = new KDLObject();
    jntArray_ = KDL::JntArray(n_);
    jntVel_ = KDL::JntArray(n_);
    coriol_ = KDL::JntArray(n_);
    dynParam_ = new KDL::ChainDynParam(chain_,KDL::Vector(0,0,-9.81));
    jacSol_ = new KDL::ChainJntToJacSolver(chain_);
    jntJacDotSol_ = new KDL::ChainJntToJacDotSolver(chain_);
    fkSol_ = new KDL::ChainFkSolverPos_recursive(chain_);
    fkVelSol_ = new KDL::ChainFkSolverVel_recursive(chain_);
    idSolver_ = new KDL::ChainIdSolver_RNE(chain_,KDL::Vector(0,0,-9.81));
    jsim_.resize(n_);
    grav_.resize(n_);
    q_min_.data.resize(n_);
    q_max_.data.resize(n_);
    q_min_.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96; //-2*M_PI,-2*M_PI;//
    q_max_.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96; //2*M_PI, 2*M_PI;
}

void KDLRobot::update(std::vector<double> _jnt_values, std::vector<double> _jnt_vel)
{
    KDL::Twist s_T_f;
    KDL::Frame s_F_f;
    KDL::Jacobian s_J_f(7);
    KDL::Jacobian s_J_dot_f(7);
    KDL::FrameVel s_Fv_f;
    KDL::JntArrayVel jntVel(jntArray_,jntVel_);
    KDL::Twist s_J_dot_q_dot_f;

    // joints space
    updateJnts(_jnt_values, _jnt_vel);
    dynParam_->JntToMass(jntArray_, jsim_);
    dynParam_->JntToCoriolis(jntArray_, jntVel_, coriol_);
    dynParam_->JntToGravity(jntArray_, grav_);

    // robot flange
    fkVelSol_->JntToCart(jntVel, s_Fv_f);
    s_T_f = s_Fv_f.GetTwist();
    s_F_f = s_Fv_f.GetFrame();
    int err = jacSol_->JntToJac(jntArray_, s_J_f);
    err = jntJacDotSol_->JntToJacDot(jntVel, s_J_dot_q_dot_f);
    err = jntJacDotSol_->JntToJacDot(jntVel, s_J_dot_f);

    // robot end-effector
    s_F_ee_ = s_F_f*f_F_ee_;
    KDL::Vector s_p_f_ee = s_F_ee_.p - s_F_f.p;
    KDL::changeRefPoint(s_J_f, s_p_f_ee, s_J_ee_);
    KDL::changeRefPoint(s_J_dot_f, s_p_f_ee, s_J_dot_ee_);
    KDL::changeBase(s_J_ee_, s_F_ee_.M.Inverse(), b_J_ee_);
    KDL::changeBase(s_J_dot_ee_, s_F_ee_.M.Inverse(), b_J_dot_ee_);
    s_V_ee_ = s_T_f.RefPoint(s_p_f_ee);

    // object
    obj_->setFrame(s_F_ee_*ee_F_obj_);
    KDL::Vector s_p_ee_obj = obj_->getFrame().p - s_F_ee_.p;
    s_V_obj_ = s_V_ee_.RefPoint(s_p_ee_obj);
    b_V_obj_ = adjoint(obj_->getFrame().Inverse(), s_V_obj_);
    obj_->setBodyVelocity(b_V_obj_);
    obj_->setSpatialVelocity(s_V_obj_);
    KDL::changeRefPoint(s_J_ee_, s_p_ee_obj, s_J_obj_);
    KDL::changeRefPoint(s_J_dot_ee_, s_p_ee_obj, s_J_dot_obj_);
    KDL::changeBase(s_J_obj_, obj_->getFrame().M.Inverse(), b_J_obj_);
    KDL::changeBase(s_J_dot_obj_, obj_->getFrame().M.Inverse(), b_J_dot_obj_);

    //contacts
    for (unsigned int i = 0; i < contacts_.size(); i++)
    {
        // update contacts jacobian
        KDL::Jacobian J(n_);
        KDL::changeRefFrame(b_J_ee_, contacts_[i].getFrame().Inverse(), J);
        // KDL::changeBase(J, obj_->getFrame().M.Inverse(), J);
        Eigen::Matrix<double,3,7> J_ci = obj_->getContacts().at(0).getB().transpose()*J.data;
        b_J_c_.at(i) = J_ci;
    }

    //    std::cout << "s_F_f: " << std::endl << s_Fv_f.GetFrame() << std::endl;
    //    std::cout << "s_F_ee: " << std::endl << s_F_ee_ << std::endl;
    //    std::cout << "s_F_obj: " << std::endl << obj_->getFrame() << std::endl;
    //    std::cout << "s_J_ee: " << std::endl << s_J_ee_.data << std::endl;
    //    std::cout << "b_J_ee: " << std::endl << b_J_ee_.data << std::endl;
    //    std::cout << "s_J_obj: " << std::endl << s_J_obj_.data << std::endl;
    //    std::cout << "b_J_obj: " << std::endl << b_J_obj_.data << std::endl;
    //    for (int i = 0; i < contacts_.size(); i++)
    //    {
    //        std::cout << "b_J_c_" << i << ": " << std::endl << b_J_c_.at(i) << std::endl;
    //    }
    //    std::cout << "s ee vel: "<< cart_vel_.vel << " "<< cart_vel_.rot << std::endl;
    //    KDL::MultiplyJacobian(jac_,jntVel_,cart_vel_);
    //    std::cout << "jacobian: "<< jac_.data << std::endl;
    //    std::cout << "jac dot: "<< jacDotqDot_.vel << " "<< jacDotqDot_.rot << std::endl;
    //    std::cout << "cart vel: "<< cart_vel_.vel << " "<< cart_vel_.rot << std::endl;

    //    std::cout << "fk Vel result: " << fkVelSol_->strError(err) << std::endl;
    //    std::cout << "jacobian result: " << jacSol_->strError(err) << std::endl;
    //    std::cout << "jacobian dot result: " << jntJacDotSol_->strError(err) << std::endl;
    //    std::cout << "jacobian: "<< jac_.data << std::endl;
    //    std::cout << "jac dot: "<< jacDotqDot_.vel << " "<< jacDotqDot_.rot << std::endl;
    //    std::cout << "cart vel: "<< cart_vel_.vel << " "<< cart_vel_.rot << std::endl;

}


////////////////////////////////////////////////////////////////////////////////
//                                 CHAIN                                      //
////////////////////////////////////////////////////////////////////////////////

void KDLRobot::createChain(KDL::Tree &robot_tree)
{

    //if(!robot_tree.getChain(robot_tree.getRootSegment()->first, "link3", chain_))
    //if(!robot_tree.getChain(robot_tree.getRootSegment()->first, "lbr_iiwa_link_7",chain_))
    if(!robot_tree.getChain(robot_tree.getRootSegment()->first, "planar_end_eff_link", chain_))
    {
        std::cout << "Failed to create KDL robot" << std::endl;
        return;
    }
    std::cout << "KDL robot model created" << std::endl;
    std::cout << "with " << chain_.getNrOfJoints() << " joints" << std::endl;
    std::cout << "and " << chain_.getNrOfSegments() << " segments" << std::endl;
}

unsigned int KDLRobot::getNrJnts()
{
    return n_;
}

unsigned int KDLRobot::getNrSgmts()
{
    return chain_.getNrOfSegments();
}

////////////////////////////////////////////////////////////////////////////////
//                                 JOINTS                                     //
////////////////////////////////////////////////////////////////////////////////

void KDLRobot::updateJnts(std::vector<double> _jnt_pos, std::vector<double> _jnt_vel)
{
    for (unsigned int i = 0; i < n_; i++)
    {
        //std::cout << _jnt_pos[i] << std::endl;
        jntArray_(i) = _jnt_pos[i];
        jntVel_(i) = _jnt_vel[i];
    }
}
Eigen::VectorXd KDLRobot::getJntValues()
{
    return jntArray_.data;
}

Eigen::VectorXd KDLRobot::getJntVelocities()
{
    return jntVel_.data;
}

Eigen::MatrixXd KDLRobot::getJntLimits()
{
    Eigen::MatrixXd jntLim;
    jntLim.resize(n_,2);

    jntLim.col(0) = q_min_.data;
    jntLim.col(1) = q_max_.data;

    return jntLim;
}

Eigen::MatrixXd KDLRobot::getJsim()
{
    return jsim_.data;
}

Eigen::VectorXd KDLRobot::getCoriolis()
{
    return coriol_.data;
}

Eigen::VectorXd KDLRobot::getGravity()
{
    return grav_.data;
}

Eigen::VectorXd KDLRobot::getID(const KDL::JntArray &q,
                                const KDL::JntArray &q_dot,
                                const KDL::JntArray &q_dotdot,
                                const KDL::Wrenches &f_ext)
{
    Eigen::VectorXd t;
    t.resize(chain_.getNrOfJoints());
    KDL::JntArray torques(chain_.getNrOfJoints());
    int r = idSolver_->CartToJnt(q,q_dot,q_dotdot,f_ext,torques);
    std::cout << "idSolver result: " << idSolver_->strError(r) << std::endl;
    // std::cout << "torques: " << torques.data.transpose() << std::endl;
    t = torques.data;
    return t;
}

////////////////////////////////////////////////////////////////////////////////
//                              END-EFFECTOR                                  //
////////////////////////////////////////////////////////////////////////////////

KDL::Frame KDLRobot::getEEFrame()
{
    return s_F_ee_;
}

KDL::Frame KDLRobot::getEEObj()
{
    return ee_F_obj_;
}

KDL::Twist KDLRobot::getEEVelocity()
{
    return s_V_ee_;
}

KDL::Twist KDLRobot::getEEBodyVelocity()
{
    return s_V_ee_;
}

KDL::Jacobian KDLRobot::getEEJacobian()
{
    return s_J_ee_;
}

KDL::Jacobian KDLRobot::getEEBodyJacobian()
{
    //    KDL::Frame ee_F_s = this->getEEPose().Inverse();
    //    KDL::Vector pkdl = ee_F_s.p;
    //    KDL::Rotation M = ee_F_s.M;
    //    std::cout << adjoint(toEigen(pkdl),toEigen(M))*s_J_ee_.data << std::endl;
    //    s_J_ee_.changeRefFrame(ee_F_s);
    //    std::cout << s_J_ee_.data << std::endl;
    //    return adjoint(toEigen(pkdl),toEigen(M))*s_J_ee_.data;
    return b_J_ee_;
}

Eigen::VectorXd KDLRobot::getEEJacDotqDot()
{
    return s_J_dot_ee_.data;
}

void KDLRobot::addEE(const KDL::Frame &_f_F_ee)
{
    f_F_ee_ = _f_F_ee;
    this->update(toStdVector(this->jntArray_.data), toStdVector(this->jntVel_.data));
}

////////////////////////////////////////////////////////////////////////////////
//                              CONTACTS                                      //
////////////////////////////////////////////////////////////////////////////////

std::vector<Eigen::Matrix<double,3,7>> KDLRobot::getContactsBodyJac() const
{
    return b_J_c_;
}

void KDLRobot::setContacts(const std::vector<Contact> &_contact)
{
    contacts_.resize(_contact.size());
    for (unsigned int i = 0; i < _contact.size(); i++)
        contacts_.at(i) = _contact.at(i);
}

std::vector<Contact> KDLRobot::getContacts() const
{
    return contacts_;
}

void KDLRobot::setContactsWrenches(const std::vector<KDL::Wrench> &_w)
{
    for (unsigned int i = 0; i < _w.size(); i++)
        contacts_.at(i).setWrench(_w.at(i));
}

////////////////////////////////////////////////////////////////////////////////
//                                OBJECT                                      //
////////////////////////////////////////////////////////////////////////////////

void KDLRobot::addObj(KDLObject &_obj)
{
    obj_= &_obj;
    ee_F_obj_ = s_F_ee_.Inverse()*obj_->getFrame();

    std::cout << "s_T_obj: " << std::endl << obj_->getFrame() << std::endl;
    std::cout << "ee_T_obj: " << std::endl << ee_F_obj_ << std::endl;

    // compute contacts on the robot in the ee frame
    std::vector<Contact> robotContacts(obj_->getContacts().size());

    for (unsigned int i = 0; i < obj_->getContacts().size(); i++)
        robotContacts.at(i).setFrame(this->getEEFrame().Inverse()*obj_->getFrame()*obj_->getContacts()[i].getFrame());
    this->setContacts(robotContacts);

    std::cout << "added object with contact points: " << std::endl;
    for (unsigned int i = 0; i < this->getContacts().size(); i++)
        std::cout << "ee_T_c "<< i << std::endl << contacts_[i].getFrame() << std::endl;

    this->update(toStdVector(this->jntArray_.data), toStdVector(this->jntVel_.data));
}

KDL::Jacobian KDLRobot::getObjJac() const
{
    return s_J_obj_;
}

KDL::Jacobian KDLRobot::getObjBodyJac() const
{
    return b_J_obj_;
}

KDL::Jacobian KDLRobot::getObjJacDot() const
{
    return s_J_dot_obj_;
}

KDL::Jacobian KDLRobot::getObjBodyJacDot() const
{
    return b_J_dot_obj_;
}
