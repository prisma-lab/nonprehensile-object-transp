#include "kdl_ros_control/kdl_object.h"

int main(int argc, char **argv)
{
    double frictionCoeff = 0.2, m = 0.1;
    Eigen::Matrix3d I = 1e-6*Eigen::Matrix3d::Identity();
    std::vector<KDL::Frame> contacts(4);
    contacts.at(0).p = KDL::Vector(-0.05, 0.05,-0.05);
    contacts.at(1).p = KDL::Vector(0.05, 0.05,-0.05);
    contacts.at(2).p = KDL::Vector(0.05,-0.05,-0.05);
    contacts.at(3).p = KDL::Vector(-0.05,-0.05,-0.05);
    KDLObject obj(m, I, contacts, frictionCoeff);

    KDL::Wrench F_b = KDL::Wrench(KDL::Vector(0,0,10), KDL::Vector(0,0,0));
    obj.computeContactForces(F_b);


//    obj.setV_b(KDL::Twist(KDL::Vector(0,1,1),KDL::Vector(1,0,0)));
//    obj.setFrame(KDL::Frame(KDL::Rotation::Rot(KDL::Vector(1,0,0),M_PI/6.0),KDL::Vector(0,0,0)));
//    F_b = obj.computeID();
//    std::cout << "ID: " << F_b.force.x() << " "
//              << F_b.force.y() << " "
//              << F_b.force.z() << " "
//              << F_b.torque.x() << " "
//              << F_b.torque.y() << " "
//              << F_b.torque.z() << " "
//              << std::endl;
    return 0;
}
