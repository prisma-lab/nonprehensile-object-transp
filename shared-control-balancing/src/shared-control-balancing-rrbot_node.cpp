#include <stdio.h>
#include <sstream>

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"

#define SAVE_DATA 1

#if SAVE_DATA
std::ofstream path_file("path.txt");
std::ofstream velocity_path_file("path_velocity.txt");
std::ofstream accel_path_file("path_accel.txt");
std::ofstream robot_file("robot.txt");
std::ofstream velocity_robot_file("robot_velocity.txt");
std::ofstream torque_robot_file("robot_torque.txt");
std::ofstream jnt_pos_d_robot_file("robot_jnt_pos_d.txt");
std::ofstream jnt_vel_d_robot_file("robot_jnt_vel_d.txt");
std::ofstream jnt_acc_d_robot_file("robot_jnt_acc_d.txt");
std::ofstream jnt_pos_robot_file("robot_jnt_pos.txt");
std::ofstream jnt_vel_robot_file("robot_jnt_vel.txt");
std::ofstream jnt_acc_robot_file("robot_jnt_acc.txt");
#endif

std::vector<double> jnt_pos;
std::vector<double> jnt_vel;
bool jnt_values_available = false;

void jointStateCallback(const sensor_msgs::JointState & msg)
{
    jnt_pos.clear();
    jnt_vel.clear();
    for (int i = 0; i < msg.position.size(); i++)
    {
        jnt_pos.push_back(msg.position[i]);
        jnt_vel.push_back(msg.velocity[i]);
        std::cout << "joint " << i << " position = " <<  jnt_pos[i] << std::endl;
    }
    if (!jnt_values_available)
        jnt_values_available = true;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kdl_ros_control_node");
    ros::NodeHandle n;

    ros::Subscriber joint_state_pub = n.subscribe("/rrbot/joint_states", 1000, jointStateCallback);
    ros::Publisher joint1_effort_pub = n.advertise<std_msgs::Float64>("/rrbot/joint1_effort_controller/command", 1);
    ros::Publisher joint2_effort_pub = n.advertise<std_msgs::Float64>("/rrbot/joint2_effort_controller/command", 1);

    ros::Rate loop_rate(1000);
    KDL::Tree robot_tree;

    std::string robot_desc_string;
    n.param("robot_description", robot_desc_string, std::string());
    if (!kdl_parser::treeFromString(robot_desc_string, robot_tree)){
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }

    KDLRobot robot(robot_tree);
    KDL::Frame j_T_ee = KDL::Frame::Identity();
    j_T_ee.p = KDL::Vector(0,0,0.95);
    robot.addEndEffector(j_T_ee);

    KDL::JntArray zero(robot.getNrJnts());
    zero.data.setZero();
    KDL::JntArray qd(robot.getNrJnts()),dqd(robot.getNrJnts()),ddqd(robot.getNrJnts());

    Eigen::VectorXd tau;
    tau.resize(robot.getNrJnts());
    std_msgs::Float64 tau1_msg, tau2_msg;

    KDLController controller_(robot);
    KDLPlanner planner(1,1);

    // Wait for joint values
    while(!jnt_values_available)
    {
        ROS_INFO("Joint state not available yet ...");
        ros::spinOnce();
    }

    robot.update(jnt_pos, jnt_vel);
    KDL::Frame T_b = KDL::Frame::Identity();
    T_b.p = KDL::Vector(0,0,1.95);
    KDL::Frame T_ee_i = T_b.Inverse()*robot.getCartesianPose();
    KDL::Frame des_pose = T_ee_i;
    KDL::Twist des_cart_vel = KDL::Twist::Zero();
    KDL::Twist des_cart_acc = KDL::Twist::Zero();

    double t = 0, traj_dur = 2.0, traj_acc = 0.5;
    double deltat = 0.001;
    Eigen::Vector3d p_i(T_ee_i.p.x(), T_ee_i.p.y(), T_ee_i.p.z());
    Eigen::Vector3d p_e(T_ee_i.p.x(), T_ee_i.p.y(), T_ee_i.p.z()+1.0);
    trajectory_point p;

    while (ros::ok())
    {
        if (jnt_values_available)
        {
//            std::vector<double> jnt_pos_;
//            std::vector<double> jnt_vel_;
//            for (int i = 0; i < 2; i++)
//            {
//                jnt_pos_.push_back(qd.data[i]);
//                jnt_vel_.push_back(dqd.data[i]);
//            }
            robot.update(jnt_pos, jnt_vel);
            // std::cout << "p_i: " << p_i << std::endl;
            // std::cout << "p_e: " << p_e << std::endl;
            std::cout << "T_ee: " << robot.getCartesianPose() << std::endl;
            if (t <= traj_dur)
            {
                p = planner.compute_trajectory(t,traj_dur,traj_acc,p_i,p_e);
                des_cart_vel.vel = toKDL(p.vel);
                des_cart_acc.vel = toKDL(p.acc);
            }
            else
            {
                des_cart_vel = KDL::Twist::Zero();
                des_cart_acc = KDL::Twist::Zero();
            }

            des_pose.p = toKDL(p.pos);
            robot.getInverseKinematicsTwoLinks(p.pos,p.vel,p.acc,qd,dqd,ddqd);
            std::cout << "qd :" << qd.data.transpose() << std::endl;

            tau = controller_.idCntr(qd, dqd, ddqd, 100, 25);
            std::cout << "tau :" << tau.transpose() << std::endl;

//            KDL::Wrenches w;
//            w.resize(3);
//            tau = robot.getID(qd,dqd,ddqd,w);
//            std::cout << "tau :" << tau.transpose() << std::endl;
            tau1_msg.data = tau[0];
            tau2_msg.data = tau[1];
            joint1_effort_pub.publish(tau1_msg);
            joint2_effort_pub.publish(tau2_msg);
            t = t + deltat;


#if SAVE_DATA
            double ad,bd,cd;
            double a,b,c;
            KDL::Frame T = robot.getCartesianPose();
            T.M.GetEulerZYX(a,b,c);
            KDL::Twist V = robot.getCartesianVelocity();
            Eigen::VectorXd jnt_pos = robot.getJntValues();
            Eigen::VectorXd jnt_vel = robot.getJntVelocities();
            KDL::Frame T_d = T_b*des_pose;

            T_d.M.GetEulerZYX(ad,bd,cd);
            path_file << T_d.p.x() << " "
                      << T_d.p.y() << " "
                      << T_d.p.z() << " "
                      << ad << " "
                      << bd << " "
                      << cd << "\n";

            velocity_path_file << des_cart_vel.vel.x() << " "
                               << des_cart_vel.vel.y() << " "
                               << des_cart_vel.vel.z() << " "
                               << des_cart_vel.rot.x() << " "
                               << des_cart_vel.rot.y() << " "
                               << des_cart_vel.rot.z() << "\n";

            accel_path_file << des_cart_acc.vel.x() << " "
                            << des_cart_acc.vel.y() << " "
                            << des_cart_acc.vel.z() << " "
                            << des_cart_acc.rot.x() << " "
                            << des_cart_acc.rot.y() << " "
                            << des_cart_acc.rot.z() << "\n";

            robot_file << T.p.x() << " "
                       << T.p.y() << " "
                       << T.p.z() << " "
                       << a << " "
                       << b << " "
                       << c << "\n";

            velocity_robot_file << V.vel.x() << " "
                                << V.vel.y() << " "
                                << V.vel.z() << " "
                                << V.rot.x() << " "
                                << V.rot.y() << " "
                                << V.rot.z() << "\n";

            torque_robot_file << tau[0] << " "
                                        << tau[1]
                                        << "\n";

            jnt_pos_d_robot_file << qd.data[0] << " "
                                               << qd.data[1]
                                               << "\n";

            jnt_vel_d_robot_file << dqd.data[0] << " "
                                                << dqd.data[1]
                                                << "\n";

            jnt_acc_d_robot_file << ddqd.data[0] << " "
                                                 << ddqd.data[1]
                                                 << "\n";

            jnt_pos_robot_file << jnt_pos[0] << " "
                                             << jnt_pos[1]
                                             << "\n";

            jnt_vel_robot_file << jnt_vel[0] << " "
                                             << jnt_vel[1]
                                             << "\n";

            path_file.flush();
            velocity_path_file.flush();
            accel_path_file.flush();
            robot_file.flush();
            velocity_robot_file.flush();
            torque_robot_file.flush();
            jnt_pos_d_robot_file.flush();
            jnt_vel_d_robot_file.flush();
            jnt_acc_d_robot_file.flush();
            jnt_pos_robot_file.flush();
            jnt_vel_robot_file.flush();
            jnt_acc_robot_file.flush();

#endif
        }

        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}
