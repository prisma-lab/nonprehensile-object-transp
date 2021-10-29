#ifndef KDLOBJECT
#define KDLOBJECT

#include "Eigen/Dense"
#include "kdl/frames.hpp"
#include "kdl/jacobian.hpp"
#include "kdl_ros_control/alglib/optimization.h"

class Contact
{

public:

    Contact(); // pass values to constructor

    KDL::Frame getFrame() const;
    KDL::Wrench getWrench() const;    
    Eigen::Matrix<double, 3, 4> getConeVersors() const;
    Eigen::Matrix<double, 6, 3> getB() const;
    Eigen::Matrix<double, 6, 3> getBo() const;
    double getTheta() const;

    void setFrame(const KDL::Frame &value);
    void setMu(const double _mu);
    void setWrench(const KDL::Wrench &value);
    void setTwist(const KDL::Twist &value);

private:

    KDL::Frame F_;
    KDL::Twist T_;
    KDL::Wrench W_;
    Eigen::Matrix<double,6,3> B_;
    Eigen::Matrix<double,6,3> Bo_;
    Eigen::Matrix<double,3,4> Fc_hat_;
    Eigen::Vector3d z_hat_;
    double mu_, theta_;
};

class KDLObject
{

public:

    KDLObject();
    KDLObject(double &m, Eigen::Matrix3d &I, std::vector<KDL::Frame> &c, double &mu);

    double getMass() const;
    KDL::Frame getFrame() const;
    KDL::Wrench getBodyWrench() const;
    KDL::Twist getBodyVelocity() const;
    KDL::Twist getSpatialVelocity() const;
    std::vector<Contact> getContacts() const;
    std::vector<KDL::Wrench> getContactsWrenches() const;
    Eigen::Matrix<double, 6, 12> getGraspMatrix() const;
    Eigen::Matrix<double, 3, 12> getGraspMatrixOri() const;
    Eigen::Matrix<double, 3, 3> getInertia() const;
    Eigen::Matrix<double, 6, 6> getMassMatrix() const;

    void setFrame(const KDL::Frame &value);
    void setBodyWrench(const KDL::Wrench &value);
    void setBodyVelocity(const KDL::Twist &value);
    void setSpatialVelocity(const KDL::Twist &value);
    void setContactFrames(const std::vector<KDL::Frame> &value);
    void setContactWrenches(const std::vector<KDL::Wrench> &value);
    void setContactTwists(const std::vector<KDL::Twist> &value);

    void addContact(const KDL::Frame &F_bc, const double _mu);
    void computeContactForces(KDL::Wrench &F_b);
    void computeContactVelocities(KDL::Twist &V_b);
    void computeC();
    void computeN();
    KDL::Wrench computeID();
    KDL::Twist computeFD(const KDL::Wrench &Fb) const;


private:

    double mass_;
    double mu_;

    KDL::Frame F_b_;
    KDL::Wrench W_b_;
    KDL::Twist V_b_;
    KDL::Twist s_V_b_;
    Eigen::Matrix<double,6,1> N_;
    Eigen::Matrix<double,6,1> C_;
    Eigen::Matrix<double,3,1> g_;
    Eigen::Matrix<double,3,3> I_;
    Eigen::Matrix<double,6,6> massMatrix;
    Eigen::Matrix<double,6,12> G_;
    Eigen::Matrix<double,3,12> Go_;
    std::vector<Contact> contacts_;
};

#endif
