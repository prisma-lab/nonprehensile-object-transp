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
#include "std_msgs/Float32MultiArray.h"
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
std::vector<double> input_acc(3,0), input_pos(3,0);

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
void accelerometerCallback(const std_msgs::Float32MultiArray & msg)
{
    input_acc.clear();
    input_acc.push_back(msg.data[0]);
    input_acc.push_back(msg.data[1]);
    input_acc.push_back(msg.data[2]);
}

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
    ros::Subscriber accelerometer_sub = n.subscribe("/accelerometer", 1, accelerometerCallback);
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

    // Services
    ros::ServiceClient obj_set_state_srv = n.serviceClient<gazebo_msgs::SetLinkState>("/gazebo/set_link_state");
    ros::ServiceClient des_obj_set_state_srv = n.serviceClient<gazebo_msgs::SetLinkState>("/gazebo/set_link_state");
    ros::ServiceClient obj_get_dyn_srv = n.serviceClient<gazebo_msgs::GetLinkProperties>("/gazebo/get_link_properties");
    ros::ServiceClient robot_set_state_srv = n.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");
    ros::ServiceClient pauseGazebo = n.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    ros::ServiceClient unpauseGazebo = n.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");

    // Messages
    std_msgs::Float64 tau1_msg, tau2_msg, tau3_msg, tau4_msg, tau5_msg, tau6_msg, tau7_msg;

    gazebo_msgs::SetLinkState obj_state;
    obj_state.request.link_state.link_name = "cube::link_cube";
    obj_state.request.link_state.twist.linear.x = 0.0;
    obj_state.request.link_state.twist.linear.y = 0.0;
    obj_state.request.link_state.twist.linear.z = 0.0;
    obj_state.request.link_state.twist.angular.x = 0.0;
    obj_state.request.link_state.twist.angular.y = 0.0;
    obj_state.request.link_state.twist.angular.z = 0.0;
    obj_state.request.link_state.reference_frame = "world";
    obj_state.request.link_state.pose.position.x = 0.594973; // 0.564973;
    obj_state.request.link_state.pose.position.y = -0.312973; // 0.372973
    obj_state.request.link_state.pose.position.z = 0.522153;
    obj_state.request.link_state.pose.orientation.x = 0.0;
    obj_state.request.link_state.pose.orientation.y = 0.0;
    obj_state.request.link_state.pose.orientation.z = 0.0;
    if(obj_set_state_srv.call(obj_state))
        ROS_INFO("Object state set.");
    else
        ROS_INFO("Failed to set object state.");

    gazebo_msgs::SetLinkState des_obj_state = obj_state;
    des_obj_state.request.link_state.link_name = "des_cube::des_cube";
    if(des_obj_set_state_srv.call(des_obj_state))
        ROS_INFO("Desired object state set.");
    else
        ROS_INFO("Failed to set desired object state.");

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
    obj_dyn_prop.request.link_name = "cube::link_cube";
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

    // Create controller
    KDLController controller_(robot, obj);

    // Gains
    double Kp = 50, Kd = sqrt(Kp);

#if SAVE_DATA
    std::string robot_file_name = "robot_" + std::to_string(i*spatial_step) + "_" + std::to_string(j*spatial_step) + ".txt";
    std::string path_file_name = "path_" + std::to_string(i*spatial_step) + "_" + std::to_string(j*spatial_step) + ".txt";
    std::string obj_file_name = "obj_" + std::to_string(i*spatial_step) + "_" + std::to_string(j*spatial_step) + ".txt";

    std::ofstream robot_file(robot_file_name);
    std::ofstream path_file(path_file_name);
    std::ofstream obj_file(obj_file_name);
#endif

    // Set desired object state
    KDL::Frame des_obj_pose = KDL::Frame::Identity();
    KDL::Twist des_obj_vel = KDL::Twist::Zero();
    KDL::Twist des_obj_acc = KDL::Twist::Zero();
    input_pos[0] = obj_state.request.link_state.pose.position.x;
    input_pos[1] = obj_state.request.link_state.pose.position.y;
    input_pos[2] = obj_state.request.link_state.pose.position.z;

    // Retrieve initial simulation time
    ROS_INFO_STREAM_ONCE("Starting control loop ...");

    double velocity_gain = 0.0005;
    while (true)
    {
        //unpauseGazebo.call(pauseSrv);
        if (robot_state_available)
        {
            // Update robot
            robot.update(jnt_pos, jnt_vel);

            // Extract desired pose
            input_pos[0] += input_acc[0]*velocity_gain;
            input_pos[1] += input_acc[1]*velocity_gain;

            des_obj_pose.p = KDL::Vector(input_pos[0], input_pos[1], input_pos[2]);
            des_obj_vel = KDL::Twist(KDL::Vector(input_acc[0]*velocity_gain, input_acc[1]*velocity_gain, 0.0), KDL::Vector::Zero());

#if USE_SHARED_CNTR
            tau = controller_.sharedCntr(des_obj_pose, des_obj_vel, des_obj_acc, Kp, Kd);
#endif //USE_SHARED_CNTR

            // Set torques
            tau1_msg.data = tau[0];
            tau2_msg.data = tau[1];
            tau3_msg.data = tau[2];
            tau4_msg.data = tau[3];
            tau5_msg.data = tau[4];
            tau6_msg.data = tau[5];
            tau7_msg.data = tau[6];
            joint1_effort_pub.publish(tau1_msg);
            joint2_effort_pub.publish(tau2_msg);
            joint3_effort_pub.publish(tau3_msg);
            joint4_effort_pub.publish(tau4_msg);
            joint5_effort_pub.publish(tau5_msg);
            joint6_effort_pub.publish(tau6_msg);
            joint7_effort_pub.publish(tau7_msg);

            // Set desired object pose
            des_obj_state.request.link_state.pose.position.x = input_pos[0];
            des_obj_state.request.link_state.pose.position.y = input_pos[1];
            if(des_obj_set_state_srv.call(des_obj_state))
                ROS_INFO("Desired pose state set.");
            else
                ROS_INFO("Failed to set desired pose state.");

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

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
