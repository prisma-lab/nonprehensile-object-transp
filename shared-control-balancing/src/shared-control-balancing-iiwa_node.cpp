#include <stdio.h>
#include <fstream>
#include <sstream>
#include <string.h>

#include "kdl_ros_control/kdl_robot.h"
#include "kdl_ros_control/kdl_control.h"
#include "kdl_ros_control/kdl_object.h"
#include "kdl_ros_control/kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"
#include "kdl_ros_control/utils.h"

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "gazebo_msgs/GetLinkProperties.h"
#include "gazebo_msgs/SetLinkState.h"
#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/SetModelConfiguration.h"
#include <std_srvs/Empty.h>

#define USE_SHARED_CNTR 1
#define USE_JNT_ID 1
#define DEBUG 0
#define SAVE_DATA 0

// Global variables
std::vector<double> jnt_pos(7,0.0), jnt_vel(7,0.0), obj_pos(6,0.0),  obj_vel(6,0.0);
bool robot_state_available = false, obj_state_available = false;
double obj_mass;
Eigen::Matrix3d obj_inertia = Eigen::Matrix3d::Identity();

// Functions
KDLRobot createRobot(ros::NodeHandle &_n)
{
    KDL::Tree robot_tree;
    std::string robot_desc_string;
    _n.param("robot_description", robot_desc_string, std::string());
    if (!kdl_parser::treeFromString(robot_desc_string, robot_tree)){
        ROS_ERROR("Failed to construct kdl tree");
    }
    KDLRobot robot(robot_tree);
    return robot;
}


// Callbacks
void objectStateCallback(const gazebo_msgs::LinkStates & msg)
{
    obj_pos.clear();
    obj_vel.clear();

    obj_pos.push_back(msg.pose[1].position.x);
    obj_pos.push_back(msg.pose[1].position.y);
    obj_pos.push_back(msg.pose[1].position.z);
    obj_pos.push_back(msg.pose[1].orientation.x);
    obj_pos.push_back(msg.pose[1].orientation.y);
    obj_pos.push_back(msg.pose[1].orientation.z);

    obj_vel.push_back(msg.twist[1].linear.x);
    obj_vel.push_back(msg.twist[1].linear.y);
    obj_vel.push_back(msg.twist[1].linear.z);
    obj_vel.push_back(msg.twist[1].angular.x);
    obj_vel.push_back(msg.twist[1].angular.y);
    obj_vel.push_back(msg.twist[1].angular.z);

    obj_state_available = true;
}

void jointStateCallback(const sensor_msgs::JointState & msg)
{
    robot_state_available = true;
    jnt_pos.clear();
    jnt_vel.clear();
    for (int i = 0; i < msg.position.size(); i++)
    {
        jnt_pos.push_back(msg.position[i]);
        jnt_vel.push_back(msg.velocity[i]);
    }
}

// Main
int main(int argc, char **argv)
{
    // Init node
    ros::init(argc, argv, "kdl_ros_control_node");
    ros::NodeHandle n;

    // Rate
    ros::Rate loop_rate(500);

    // Subscribers
    ros::Subscriber joint_state_sub = n.subscribe("/lbr_iiwa/joint_states", 1, jointStateCallback);
    ros::Subscriber object_state_sub = n.subscribe("/gazebo/link_states", 1, objectStateCallback);

    // Publishers
    ros::Publisher joint1_effort_pub = n.advertise<std_msgs::Float64>("/lbr_iiwa/lbr_iiwa_joint_1_effort_controller/command", 1);
    ros::Publisher joint2_effort_pub = n.advertise<std_msgs::Float64>("/lbr_iiwa/lbr_iiwa_joint_2_effort_controller/command", 1);
    ros::Publisher joint3_effort_pub = n.advertise<std_msgs::Float64>("/lbr_iiwa/lbr_iiwa_joint_3_effort_controller/command", 1);
    ros::Publisher joint4_effort_pub = n.advertise<std_msgs::Float64>("/lbr_iiwa/lbr_iiwa_joint_4_effort_controller/command", 1);
    ros::Publisher joint5_effort_pub = n.advertise<std_msgs::Float64>("/lbr_iiwa/lbr_iiwa_joint_5_effort_controller/command", 1);
    ros::Publisher joint6_effort_pub = n.advertise<std_msgs::Float64>("/lbr_iiwa/lbr_iiwa_joint_6_effort_controller/command", 1);
    ros::Publisher joint7_effort_pub = n.advertise<std_msgs::Float64>("/lbr_iiwa/lbr_iiwa_joint_7_effort_controller/command", 1);
    ros::ServiceClient obj_set_state_srv = n.serviceClient<gazebo_msgs::SetLinkState>("/gazebo/set_link_state");
    ros::ServiceClient des_pose_set_state_srv = n.serviceClient<gazebo_msgs::SetLinkState>("/gazebo/set_link_state");
    ros::ServiceClient obj_get_dyn_srv = n.serviceClient<gazebo_msgs::GetLinkProperties>("/gazebo/get_link_properties");
    ros::ServiceClient robot_set_state_srv = n.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");
    ros::ServiceClient resetGazeboSimulation = n.serviceClient<std_srvs::Empty>("/gazebo/reset_simulation");
    ros::ServiceClient pauseGazebo = n.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    ros::ServiceClient unpauseGazebo = n.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");

    // Messages
    std_msgs::Float64 tau1_msg, tau2_msg, tau3_msg, tau4_msg, tau5_msg, tau6_msg, tau7_msg;

    // Services
//    std_srvs::Empty reset_simulation_srv;
//    if(resetGazeboSimulation.call(reset_simulation_srv))
//        ROS_INFO("Reset simulation.");
//    else
//        ROS_INFO("Failed to reset simulation.");

    gazebo_msgs::SetLinkState obj_init_state;
    obj_init_state.request.link_state.link_name = "lbr_iiwa::cuboid_link";
    obj_init_state.request.link_state.twist.linear.x = 0.0;
    obj_init_state.request.link_state.twist.linear.y = 0.0;
    obj_init_state.request.link_state.twist.linear.z = 0.0;
    obj_init_state.request.link_state.twist.angular.x = 0.0;
    obj_init_state.request.link_state.twist.angular.y = 0.0;
    obj_init_state.request.link_state.twist.angular.z = 0.0;
    obj_init_state.request.link_state.reference_frame = "world";
    obj_init_state.request.link_state.pose.position.x = 0.594973; // 0.564973;
    obj_init_state.request.link_state.pose.position.y = -0.312973; // 0.372973
    obj_init_state.request.link_state.pose.position.z = 0.522153;
    obj_init_state.request.link_state.pose.orientation.x = 0.0;
    obj_init_state.request.link_state.pose.orientation.y = 0.0;
    obj_init_state.request.link_state.pose.orientation.z = 0.0;
    if(obj_set_state_srv.call(obj_init_state))
        ROS_INFO("Object state set.");
    else
        ROS_INFO("Failed to set object state.");

    /*gazebo_msgs::SetLinkState des_pose_init_state;
    des_pose_init_state.request.link_state.link_name = "lbr_iiwa::cuboid_link";
    des_pose_init_state.request.link_state.reference_frame = "world";
    des_pose_init_state.request.link_state.pose.position.x = 0.594973; // 0.564973;
    des_pose_init_state.request.link_state.pose.position.y = -0.312973; // 0.372973
    des_pose_init_state.request.link_state.pose.position.z = 0.522153;
    des_pose_init_state.request.link_state.pose.orientation.x = 0.0;
    des_pose_init_state.request.link_state.pose.orientation.y = 0.0;
    des_pose_init_state.request.link_state.pose.orientation.z = 0.0;
    if(des_pose_set_state_srv.call(des_pose_init_state))
        ROS_INFO("Desired pose state set.");
    else
        ROS_INFO("Failed to set desired pose state.");
    */
    
    gazebo_msgs::SetModelConfiguration robot_init_config;
    robot_init_config.request.model_name = "lbr_iiwa";
    robot_init_config.request.urdf_param_name = "robot_description";
    robot_init_config.request.joint_names.push_back("lbr_iiwa_joint_1");
    robot_init_config.request.joint_names.push_back("lbr_iiwa_joint_2");
    robot_init_config.request.joint_names.push_back("lbr_iiwa_joint_3");
    robot_init_config.request.joint_names.push_back("lbr_iiwa_joint_4");
    robot_init_config.request.joint_names.push_back("lbr_iiwa_joint_5");
    robot_init_config.request.joint_names.push_back("lbr_iiwa_joint_6");
    robot_init_config.request.joint_names.push_back("lbr_iiwa_joint_7");
    robot_init_config.request.joint_positions.push_back(0.0);
    robot_init_config.request.joint_positions.push_back(1.57);
    robot_init_config.request.joint_positions.push_back(-1.57);
    robot_init_config.request.joint_positions.push_back(-1.2);
    robot_init_config.request.joint_positions.push_back(1.57);
    robot_init_config.request.joint_positions.push_back(-1.57);
    robot_init_config.request.joint_positions.push_back(-0.37);
    if(robot_set_state_srv.call(robot_init_config))
        ROS_INFO("Robot state set.");
    else
        ROS_INFO("Failed to set robot state.");

    gazebo_msgs::GetLinkProperties obj_dyn_prop;
    obj_dyn_prop.request.link_name = "lbr_iiwa::cuboid_link";
    if(obj_get_dyn_srv.call(obj_dyn_prop))
    {
        ROS_INFO("Object dynamic properties retrieved.");
        obj_mass = obj_dyn_prop.response.mass;
        obj_inertia << obj_dyn_prop.response.ixx,
                obj_dyn_prop.response.ixy,
                obj_dyn_prop.response.ixz,
                obj_dyn_prop.response.ixy,
                obj_dyn_prop.response.iyy,
                obj_dyn_prop.response.iyz,
                obj_dyn_prop.response.ixz,
                obj_dyn_prop.response.iyz,
                obj_dyn_prop.response.izz;
        std::cout << "Object mass: " << obj_mass << std::endl;
        std::cout << "Object inertia: " << std::endl << obj_inertia << std::endl;
    }
    else
        ROS_INFO("Failed to get object dynamic properties.");

    std_srvs::Empty pauseSrv;


    // Wait for robot and object state
    // unpauseGazebo.call(pauseSrv);
    while(!robot_state_available || !obj_state_available)
    {
        ROS_INFO_STREAM_ONCE("Robot/object state not available yet.");
        ROS_INFO_STREAM_ONCE("Please start gazebo simulation.");
        ros::spinOnce();
    }
    pauseGazebo.call(pauseSrv);

    // Create robot
    KDLRobot robot = createRobot(n);
    robot.update(jnt_pos, jnt_vel);

    // Add end-effector
    KDL::Frame f_T_ee = KDL::Frame::Identity(); f_T_ee.p[2] = 0.016; // plate height
    robot.addEE(f_T_ee);

    // Create object
    double frictionCoeff = 0.5;
    std::vector<KDL::Frame> contacts(4);
    contacts.at(0).p = KDL::Vector(-0.02, 0.02,-0.02);
    contacts.at(1).p = KDL::Vector(0.02, 0.02,-0.02);
    contacts.at(2).p = KDL::Vector(0.02,-0.02,-0.02);
    contacts.at(3).p = KDL::Vector(-0.02,-0.02,-0.02);
    KDLObject obj(obj_mass, obj_inertia, contacts, frictionCoeff);
    std::cout << toEigen(obj.getFrame().M) << std::endl;
    KDL::Wrench Fb(obj.getFrame().M.Inverse()*KDL::Vector(0,0,9.81*obj_mass), KDL::Vector(0,0,0));
    obj.computeContactForces(Fb);

    obj.setFrame(toKDL(obj_pos));

    // Add object
    robot.addObj(obj);

    // Joints
    KDL::JntArray qd(robot.getNrJnts()),dqd(robot.getNrJnts()),ddqd(robot.getNrJnts());

    // Torques
    Eigen::VectorXd tau;
    tau.resize(robot.getNrJnts());

    // Update robot
    robot.update(jnt_pos, jnt_vel);

    // Initial object frame
    KDL::Frame init_cart_pose = obj.getFrame();
    Eigen::Vector3d init_position(init_cart_pose.p.data);

    // Final object frame
    Eigen::Vector3d end_position;
    end_position << init_cart_pose.p.x(), -init_cart_pose.p.y(), init_cart_pose.p.z();

    // Plan trajectory
    double traj_duration = 2, acc_duration = 1, t = 0.0, init_time_slot = 0.0;
    KDLPlanner planner(traj_duration, acc_duration, init_position, end_position);
    trajectory_point p = planner.compute_trajectory(t);

    //momentumEstimator est(5.0, robot.getNrJnts());

    // Create controller
    KDLController controller_(robot, obj);

    // Gains
    double Kp = 50, Kd = sqrt(Kp);

    gazebo_msgs::SetLinkState obj_current_state = obj_init_state;
    KDL::Vector obj_init_pos(obj_init_state.request.link_state.pose.position.x,
                             obj_init_state.request.link_state.pose.position.y,
                             obj_init_state.request.link_state.pose.position.z);

#if SAVE_DATA
    std::string robot_file_name = "robot_" + std::to_string(i*spatial_step) + "_" + std::to_string(j*spatial_step) + ".txt";
    std::string path_file_name = "path_" + std::to_string(i*spatial_step) + "_" + std::to_string(j*spatial_step) + ".txt";
    std::string obj_file_name = "obj_" + std::to_string(i*spatial_step) + "_" + std::to_string(j*spatial_step) + ".txt";

    std::ofstream robot_file(robot_file_name);
    std::ofstream path_file(path_file_name);
    std::ofstream obj_file(obj_file_name);
#endif
    // Reset robot/object state
    // unpauseGazebo.call(pauseSrv);

//    robot_state_available = false;
//    while(!robot_state_available)
//        ros::spinOnce();

    // Retrieve initial simulation time
    ros::Time begin = ros::Time::now();
    ROS_INFO_STREAM_ONCE("Starting control loop ...");

    while ((ros::Time::now()-begin).toSec() < 2*traj_duration + init_time_slot)
    {
        //unpauseGazebo.call(pauseSrv);
        if (robot_state_available)
        {
            // Update robot
            robot.update(jnt_pos, jnt_vel);

            // Update time
            t = (ros::Time::now()-begin).toSec();

            // Extract desired pose
            KDL::Frame des_pose = KDL::Frame::Identity();
            KDL::Twist des_cart_vel = KDL::Twist::Zero(), des_cart_acc = KDL::Twist::Zero();
            if (t < init_time_slot) // wait a second
            {
                p = planner.compute_trajectory(0.0);
            }
            else if(t > init_time_slot && t <= traj_duration + init_time_slot)
            {
                p = planner.compute_trajectory(t-init_time_slot);
//                double x = p.pos.x()+ 0.1*sin(2*t);
//                p.pos.x() = x;
                des_cart_vel = KDL::Twist(KDL::Vector(p.vel[0], p.vel[1], p.vel[2]),KDL::Vector::Zero());
                des_cart_acc = KDL::Twist(KDL::Vector(p.acc[0], p.acc[1], p.acc[2]),KDL::Vector::Zero());
            }
            else
            {
                ROS_INFO_STREAM_ONCE("trajectory terminated");
            }

            des_pose.p = KDL::Vector(p.pos[0],p.pos[1],p.pos[2]);

#if USE_SHARED_CNTR
            tau = controller_.sharedCntr(des_pose, des_cart_vel, des_cart_acc, Kp, Kd);
#endif //USE_SHARED_CNTR

#if USE_JNT_ID
//            // inverse kinematics
//            des_pose=des_pose*j_T_ee.Inverse();
//            qd.data << jnt_pos[0], jnt_pos[1], jnt_pos[2], jnt_pos[3], jnt_pos[4], jnt_pos[5], jnt_pos[6];
//            robot.getInverseKinematics(des_pose,des_cart_vel,des_cart_acc,qd,dqd,ddqd);

//            // joint space inverse dynamics control
//            tau = controller_.idCntr(qd, dqd, ddqd, Kp, Kd);
#else
            double Kp = 1000;
            double Ko = 1000;
            // Cartesian space inverse dynamics control
            tau = controller_.idCntr(des_pose, des_cart_vel, des_cart_acc,
                                     Kp, Ko, 2*sqrt(Kp), 2*sqrt(Ko));
#endif

#if SAVE_DATA
            double ad,bd,cd, a,b,c;
            KDL::Frame T = robot.getCartesianPose();
            KDL::Twist V = robot.getCartesianVelocity();
            Eigen::VectorXd jnt_pos = robot.getJntValues(), jnt_vel = robot.getJntVelocities();

            des_pose.M.GetEulerZYX(ad,bd,cd);
            T.M.GetEulerZYX(a,b,c);

            path_file << des_pose.p.x() << " " << des_pose.p.y() << " " << des_pose.p.z() << " "
                      << ad << " " << bd << " " << cd << " "

                      << des_cart_vel.vel.x() << " " << des_cart_vel.vel.y() << " " << des_cart_vel.vel.z() << " "
                      << des_cart_vel.rot.x() << " " << des_cart_vel.rot.y() << " " << des_cart_vel.rot.z() << " "

                      << des_cart_acc.vel.x() << " " << des_cart_acc.vel.y() << " " << des_cart_acc.vel.z() << " "
                      << des_cart_acc.rot.x() << " " << des_cart_acc.rot.y() << " " << des_cart_acc.rot.z() << "\n";

            robot_file << T.p.x() << " " << T.p.y() << " " << T.p.z() << " "
                       << a << " " << b << " " << c << " "

                       << V.vel.x() << " " << V.vel.y() << " " << V.vel.z() << " "
                       << V.rot.x() << " " << V.rot.y() << " " << V.rot.z() << " "

                       << tau[0] << " " << tau[1] << " " << tau[2] << " "
                       << tau[3] << " " << tau[4] << " " << tau[5] << " "
                       << tau[6] << " "

                       << qd.data[0] << " " << qd.data[1] << " " << qd.data[2] << " "
                       << qd.data[3] << " " << qd.data[4] << " " << qd.data[5] << " "
                       << qd.data[6] << " "

                       << dqd.data[0] << " " << dqd.data[1] << " " << dqd.data[2] << " "
                       << dqd.data[3] << " " << dqd.data[4] << " " << dqd.data[5] << " "
                       << dqd.data[6] << " "

                       << ddqd.data[0] << " " << ddqd.data[1] << " " << ddqd.data[2] << " "
                       << ddqd.data[3] << " " << ddqd.data[4] << " " << ddqd.data[5] << " "
                       << ddqd.data[6] << " "

                       << jnt_pos[0] << " " << jnt_pos[1] << " " << jnt_pos[2] << " "
                       << jnt_pos[3] << " " << jnt_pos[4] << " " << jnt_pos[5] << " "
                       << jnt_pos[6] << " "

                       << jnt_vel[0] << " " << jnt_vel[1] << " " << jnt_vel[2] << " "
                       << jnt_vel[3] << " " << jnt_vel[4] << " " << jnt_vel[5] << " "
                       << jnt_vel[6] << "\n";

            obj_file << obj_pos[0] << " " << obj_pos[1] << " " << obj_pos[2] << " "
                                   << obj_pos[3] << " " << obj_pos[4] << " " << obj_pos[5] << " "
                                   << obj_vel[0] << " " << obj_vel[1] << " " << obj_vel[2] << " "
                                   << obj_vel[3] << " " << obj_vel[4] << " " << obj_vel[5] << "\n";

            path_file.flush();
            robot_file.flush();
            obj_file.flush();
#endif
            // Set torques
            tau1_msg.data = tau[0];
            tau2_msg.data = tau[1];
            tau3_msg.data = tau[2];
            tau4_msg.data = tau[3];
            tau5_msg.data = tau[4];
            tau6_msg.data = tau[5];
            tau7_msg.data = tau[6];

            // Publish
            joint1_effort_pub.publish(tau1_msg);
            joint2_effort_pub.publish(tau2_msg);
            joint3_effort_pub.publish(tau3_msg);
            joint4_effort_pub.publish(tau4_msg);
            joint5_effort_pub.publish(tau5_msg);
            joint6_effort_pub.publish(tau6_msg);
            joint7_effort_pub.publish(tau7_msg);

#if DEBUG
            std::cout << "jacobian: " << std::endl << robot.getJacobian() << std::endl;
            std::cout << "jsim: " << std::endl << robot.getJsim() << std::endl;
            std::cout << "c: " << std::endl << robot.getCoriolis().transpose() << std::endl;
            std::cout << "g: " << std::endl << robot.getGravity().transpose() << std::endl;
            std::cout << "qd: " << std::endl << qd.data.transpose() << std::endl;
            std::cout << "q: " << std::endl << robot.getJntValues().transpose() << std::endl;
            std::cout << "tau: " << std::endl << tau.transpose() << std::endl;
            std::cout << "desired_pose: " << std::endl << des_pose << std::endl;
            std::cout << "current_pose: " << std::endl << robot.getCartesianPose() << std::endl;
#endif

            /*des_pose_init_state.request.link_state.pose.position.x = p.pos.x();
            des_pose_init_state.request.link_state.pose.position.y = p.pos.y();
            des_pose_init_state.request.link_state.pose.position.z = p.pos.z();
            if(des_pose_set_state_srv.call(des_pose_init_state))
                ROS_INFO("Desired pose state set.");
            else
                ROS_INFO("Failed to set desired pose state.");
            */
            
            ros::spinOnce();
            loop_rate.sleep();
            // pauseGazebo.call(pauseSrv);
            // int n;
            // std::cin >> n;
        }
    }
    if(pauseGazebo.call(pauseSrv))
        ROS_INFO("Simulation paused.");
    else
        ROS_INFO("Failed to pause simulation.");


    return 0;
}


//    std::vector<KDL::Frame> trajectory_frames;
//    KDL::Frame frame_1 = KDL::Frame(KDL::Rotation::EulerZYX(0,0,0),KDL::Vector(0.45,0.2,0.5));
//    KDL::Frame frame_2 = KDL::Frame(KDL::Rotation::EulerZYX(0,1.57,0),KDL::Vector(0.65,0.2,0.5));
//    KDL::Frame frame_3 = KDL::Frame(KDL::Rotation::EulerZYX(0,0,0),KDL::Vector(0.65,-0.2,0.5));
//    trajectory_frames.push_back(frame_1);
//    trajectory_frames.push_back(frame_2);
//    trajectory_frames.push_back(frame_3);
//    planner.CreateTrajectoryFromFrames(trajectory_frames,0.1,0.1);

//    KDL::Vector center(0.0,0.0,init_cart_pose.p.z());
//    KDL::Frame end_cart_pose = KDL::Frame(
//                init_cart_pose.M,
//                KDL::Vector(init_cart_pose.p.x(),
//                            -init_cart_pose.p.y(),
//                            init_cart_pose.p.z()));
//    planner.createCircPath(init_cart_pose,
//                           center,
//                           end_cart_pose.p,
//                           end_cart_pose.M,
//                           1.57,
//                           0.02);
// trajectory_point p = planner.compute_trajectory(0.0,4.0,1.0,init_position,end_position);
//    KDL::Trajectory* traj;
//    traj = planner.getTrajectory();
