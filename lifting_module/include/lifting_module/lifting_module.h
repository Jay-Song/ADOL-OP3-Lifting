#ifndef LIFTING_MODULE_LIFTING_MODULE_H_
#define LIFTING_MODULE_LIFTING_MODULE_H_

#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <boost/thread.hpp>

#include "robotis_math/robotis_math.h"
#include "robotis_framework_common/motion_module.h"

#include "robotis_controller_msgs/StatusMsg.h"
#include "lifting_trajectory_msgs/LiftingTrajectory.h"

namespace adol
{

class LiftingModule: public robotis_framework::MotionModule,
                     public robotis_framework::Singleton<LiftingModule>
{
public:
  LiftingModule();
  virtual ~LiftingModule();

  /* ROS Framework Functions */
  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);
  void stop();
  bool isRunning();


  void onModuleEnable();
  void onModuleDisable();

  /* ROS Publish Functions */
  void publishStatusMsg(unsigned int type, std::string msg);

private:
  void queueThread();

  void getLiftingTrajectoryCallback(const lifting_trajectory_msgs::LiftingTrajectory::ConstPtr& msg);
  void getLiftingCommandCallback(const std_msgs::String::ConstPtr& msg);
  
  void getLiftingTrajectoryAngle(int trajectory_idx);

  bool computeLegInverseKinematics(double *angle, Eigen::Matrix4d& transform);
  bool computeRLegInverseKinematics(double *angle, Eigen::Matrix4d& transform);
  bool computeLLegInverseKinematics(double *angle, Eigen::Matrix4d& transform);

  boost::thread queue_thread_;
  //boost::mutex publish_mutex_;

  /* ROS Topic Publish Functions */
  ros::Publisher status_msg_pub_;

  /*  */
  lifting_trajectory_msgs::LiftingTrajectory lifting_trajectory_;

  robotis_framework::FifthOrderPolynomialTrajectory poly_;

  double joint_angle_[20];
  double initial_joint_angle_[20];
  double control_cycle_sec_;

  bool lifting_module_enabled_;

  bool init_flag_;
  bool trajectory_flag_;
  

  int trajectory_idx_;
  int trajectory_size_;

  Eigen::Matrix4d mat_rhip_to_pelvis;
  Eigen::Matrix4d mat_lhip_to_pelvis;
};

}
#endif