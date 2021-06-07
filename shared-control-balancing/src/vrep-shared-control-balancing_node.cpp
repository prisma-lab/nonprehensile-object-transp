// ROS includes.
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "Eigen/Dense"
#include "eigen_conversions/eigen_msg.h"
#include <iostream>


/*--------------------------------------------------------------------
 * Variables.
 *------------------------------------------------------------------*/

geometry_msgs::Twist force_hand;
geometry_msgs::Twist current_velocity_hand;
geometry_msgs::Pose current_hand_pose;

struct trajectory_point{
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
};

bool current_hand_pose_available = false;
bool control_loop_started = false;

/*--------------------------------------------------------------------
 * Functions.
 *------------------------------------------------------------------*/

Eigen::Matrix3d skew(const Eigen::Vector3d & t)
{
  Eigen::Matrix3d t_hat;
  t_hat <<   0,-t(2), t(1),
          t(2),    0,-t(0),
         -t(1), t(0),    0;
  return t_hat;
}

trajectory_point compute_trajectory(double time, double tf, double tc, Eigen::Vector3d traj_init, Eigen::Vector3d traj_end)
{
  /* trapezoidal velocity profile with tc acceleration time period and tf total duration.
     time = current time
     tf   = final time
     tc   = acceleration time
     traj_init = trajectory initial point
     traj_end  = trajectory final point */
  
  trajectory_point traj;

  Eigen::Vector3d ddot_traj_c = -1.0/(std::pow(tc,2)-tf*tc)*(traj_end-traj_init);

  if(time <= tc)
  {
    traj.pos = traj_init + 0.5*ddot_traj_c*std::pow(time,2);
    traj.vel = ddot_traj_c*time;
    traj.acc = ddot_traj_c;
  }
  else if(time <= tf-tc)
  {
    traj.pos = traj_init + ddot_traj_c*tc*(time-tc/2);
    traj.vel = ddot_traj_c*tc;
    traj.acc = Eigen::Vector3d::Zero();
  }
  else
  {
    traj.pos = traj_end - 0.5*ddot_traj_c*std::pow(tf-time,2);
    traj.vel = ddot_traj_c*(tf-time);
    traj.acc = -ddot_traj_c;
  }

  return traj;

}

/*--------------------------------------------------------------------
 * Callbacks.
 *------------------------------------------------------------------*/

void current_hand_pose_callback(const geometry_msgs::Pose::ConstPtr& msg)
{
  //ROS_INFO("Current hand position-> x: [%f], y: [%f], z: [%f]", msg->position.x,msg->position.y, msg->position.z);
  //ROS_INFO("Current hand orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y, msg->orientation.z, msg->orientation.w);
  
  current_hand_pose.position = msg->position;
  current_hand_pose.orientation = msg->orientation;
  
  if (!current_hand_pose_available)
    current_hand_pose_available = true;
}

void current_hand_velocity_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  current_velocity_hand.linear = msg->linear;
  current_velocity_hand.angular = msg->angular;
}


/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "shared_control_balancing_node");
  ros::NodeHandle n;

  // Declare variables.
  double x = 0.5;
  double alpha = 0.0;
  double acc_time = 1.0;
  double final_time = 4.0;
  double mass = 7.850*2; 
  ros::Time begin, now;
  trajectory_point p;
  Eigen::Vector3d traj_init_point;
  Eigen::Vector3d traj_end_point;
  Eigen::Vector3d current_position;
  Eigen::Quaterniond current_orientation;
  

  // Publisher and subscribers.
  ros::Publisher force_hand_pub = n.advertise<geometry_msgs::Twist>("cartesian_hand_force", 1);
  ros::Subscriber current_hand_pose_sub = n.subscribe("current_hand_pose", 1, current_hand_pose_callback);
  ros::Subscriber current_hand_velocity_sub = n.subscribe("current_hand_velocity", 1, current_hand_velocity_callback);

  
  // Tell ROS how fast to run this node.
  ros::Rate r(200);

  // Main loop.
  while (n.ok())
  {
    if (current_hand_pose_available)
    {
      if (!control_loop_started)
      {  
         // Retrieve initial time.
         begin = ros::Time::now();
         ROS_INFO("Initial time -> [%f]", begin.toSec());
         control_loop_started = true;

         // Retrieve initial position.
         tf::pointMsgToEigen(current_hand_pose.position, traj_init_point);
         tf::quaternionMsgToEigen(current_hand_pose.orientation, current_orientation);

         p.pos = traj_init_point;

         traj_end_point = traj_init_point;
         traj_end_point[0] = traj_end_point[0]+0.5;
      }
      
      // Retrieve current time.
      now = ros::Time::now();  
      /*ROS_INFO("Current time -> [%f]", now.toSec());
      ROS_INFO("Elapsed time -> [%f]", (now-begin).toSec()); */

      // Compute next trajectory point.
      double t = (now-begin).toSec();
      if (t <= final_time)
        p = compute_trajectory(t, final_time, acc_time, traj_init_point, traj_end_point);

      // Retrieve current robot data.
      Eigen::Matrix<double,6,1> current_velocity_eigen;
      tf::pointMsgToEigen(current_hand_pose.position, current_position);
      tf::quaternionMsgToEigen(current_hand_pose.orientation, current_orientation);
      tf::twistMsgToEigen(current_velocity_hand, current_velocity_eigen);
      Eigen::Matrix3d current_R = current_orientation.normalized().toRotationMatrix();

      // Compute linear errors.
      Eigen::Vector3d position_error = p.pos - current_position;
      Eigen::Vector3d dot_position_error = p.vel - current_velocity_eigen.block(0,0,3,1);

      // PD.
      Eigen::Matrix3d KP = 1*Eigen::Matrix3d::Identity();
      Eigen::Matrix3d KD = 1*Eigen::Matrix3d::Identity();
      Eigen::Vector3d desired_linear_force = 50*KP*position_error + 50*KD*dot_position_error;

      // Calculate desired linear acceleration
      Eigen::Vector3d desired_linear_acceleration = desired_linear_force/mass;

      // Compute desired orietation given acceleration (1D)
      double alpha = -3.14/4;//1.0/9.81*asin(desired_linear_acceleration[0]); 
      ROS_INFO("alpha desired-> [%f]", alpha);
      Eigen::Matrix3d desired_R = Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitY()).toRotationMatrix(); 
      std::cout << "desired_R -> " << desired_R << std::endl;
      // std::cout << "current_R -> " << current_R << std::endl;
      // Compute orientation errors.
      Eigen::Vector3d orientation_error;
      Eigen::Vector3d nd = desired_R.block(0,0,3,1);
      Eigen::Vector3d n = current_R.block(0,0,3,1);
      Eigen::Vector3d sd = desired_R.block(0,1,3,1);
      Eigen::Vector3d s = current_R.block(0,1,3,1);
      Eigen::Vector3d ad = desired_R.block(0,2,3,1);
      Eigen::Vector3d a = current_R.block(0,2,3,1);
      
      orientation_error = 0.5*((n).cross(nd) + (s).cross(sd) + (a).cross(ad));
      //ROS_INFO("orientation_error -> [%f],[%f],[%f]", orientation_error[0], orientation_error[1], orientation_error[2]);
      /*dot_orientation_error = orientation_error - orientation_error_prev;
      orientation_error_prev = orientation_error;*/

      // PD.
      Eigen::Matrix3d L = -0.5*(skew(nd)*skew(n)+skew(sd)*skew(s)+skew(ad)*skew(a));
      Eigen::Vector3d omega = current_velocity_eigen.block(3,0,3,1); 
      Eigen::Vector3d dot_orientation_error= -L*omega;
      Eigen::Vector3d r = Eigen::Vector3d::Zero();
      r[1] = mass*9.81;
      Eigen::Vector3d torque_slave = 0.01*KP*orientation_error + 0.01*KD*dot_orientation_error;
      Eigen::Vector3d force_slave = Eigen::Vector3d::Zero();
      //force_slave[0] = mass*9.81*sin(alpha);

      // Fill in the message.
      force_hand.linear.x = force_slave[0];
      force_hand.linear.y = force_slave[1];
      force_hand.linear.z = force_slave[2];
      force_hand.angular.x = torque_slave[0];
      force_hand.angular.y = torque_slave[1];
      force_hand.angular.z = torque_slave[2];


      // Publish the message.
      force_hand_pub.publish(force_hand);
      
    }
    else
    {
      ROS_INFO("Waiting for current robot pose...");
      ros::Duration(1).sleep();
    }

    ros::spinOnce();
    r.sleep();
  }

  return 0;
} // end main()
