#include "lifting_module/lifting_module.h"

using namespace adol;

Eigen::Matrix4d getTransformationXYZQuat(double position_x, double position_y, double position_z , double w, double x, double y, double z)
{
  Eigen::Matrix4d transformation;
  transformation << 
    1,0,0, position_x, 
    0,1,0, position_y, 
    0,0,1, position_z,
    0,0,0, 1; 
  
  Eigen::Quaterniond quat(w, x, y, z);

  transformation.block<3,3>(0,0) = quat.toRotationMatrix();

  return transformation;
}

double getSign(double num)
{
  if(num < 0)
    return -1.0;
  else
    return 1.0;
}

LiftingModule::LiftingModule()
{
  module_name_ = "lifting_module";
  control_mode_ = robotis_framework::PositionControl;

  control_cycle_sec_ = 0.008;
  // right arm
  result_["r_sho_pitch"] = new robotis_framework::DynamixelState();
  result_["r_sho_roll"] = new robotis_framework::DynamixelState();
  result_["r_el"] = new robotis_framework::DynamixelState();

  // left arm
  result_["l_sho_pitch"] = new robotis_framework::DynamixelState();
  result_["l_sho_roll"] = new robotis_framework::DynamixelState();
  result_["l_el"] = new robotis_framework::DynamixelState();

  // right leg
  result_["r_hip_yaw"] = new robotis_framework::DynamixelState();
  result_["r_hip_roll"] = new robotis_framework::DynamixelState();
  result_["r_hip_pitch"] = new robotis_framework::DynamixelState();
  result_["r_knee"] = new robotis_framework::DynamixelState();
  result_["r_ank_pitch"] = new robotis_framework::DynamixelState();
  result_["r_ank_roll"] = new robotis_framework::DynamixelState();

  // left leg
  result_["l_hip_yaw"] = new robotis_framework::DynamixelState();
  result_["l_hip_roll"] = new robotis_framework::DynamixelState();
  result_["l_hip_pitch"] = new robotis_framework::DynamixelState();
  result_["l_knee"] = new robotis_framework::DynamixelState();
  result_["l_ank_pitch"] = new robotis_framework::DynamixelState();
  result_["l_ank_roll"] = new robotis_framework::DynamixelState();

  // head
  result_["head_pan"] = new robotis_framework::DynamixelState();
  result_["head_tilt"] = new robotis_framework::DynamixelState();

  lifting_module_enabled_ = false;
  init_flag_ = false;
  trajectory_flag_ = false;
  trajectory_idx_ = -1;

  mat_rhip_to_pelvis 
    << 1, 0, 0, 0,
     0, 1, 0, 0.035,
     0, 0, 1, 0,
     0, 0, 0, 1;
  mat_lhip_to_pelvis 
   << 1, 0, 0, 0,
    0, 1, 0, -0.035,
    0, 0, 1, 0,
    0, 0, 0, 1;
}

LiftingModule::~LiftingModule()
{
  queue_thread_.join();
}

void LiftingModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_sec_ = control_cycle_msec * 0.001;
  queue_thread_ = boost::thread(boost::bind(&LiftingModule::queueThread, this));

  ros::NodeHandle ros_node;
  // Publisher
  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
    
  // init result, joint_id_table
  for (std::map<std::string, robotis_framework::Dynamixel*>::iterator it = robot->dxls_.begin();
       it != robot->dxls_.end(); it++)
  {
    std::string joint_name = it->first;
    robotis_framework::Dynamixel* dxl_info = it->second;

    //result_[joint_name] = new robotis_framework::DynamixelState();
    result_[joint_name]->goal_position_ = dxl_info->dxl_state_->goal_position_;
  }

  poly_.changeTrajectory(0, 0, 0, 0, 3.0, 1.0, 0, 0);

  lifting_module_enabled_ = false;
  init_flag_ = false;
  trajectory_flag_ = false;
  trajectory_idx_ = -1;
}

void LiftingModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors)
{
  if (enable_ == false)
    return;
  
  if (lifting_module_enabled_ == true)
  {
    for (std::map<std::string, robotis_framework::Dynamixel *>::iterator dxls_it = dxls.begin(); dxls_it != dxls.end();
         dxls_it++)
    {
      std::string joint_name = dxls_it->first;
      std::map<std::string, robotis_framework::DynamixelState *>::iterator result_it = result_.find(joint_name);
      if (result_it == result_.end())
        continue;
      else
      {
        result_it->second->goal_position_ = dxls_it->second->dxl_state_->goal_position_;
      }
    }
    lifting_module_enabled_ = false;
  }

  if (init_flag_)
  {
    double ratio =  poly_.getPosition(trajectory_idx_*control_cycle_sec_);

    //ROS_INFO_STREAM(result_["r_el"]->goal_position_ << " " << initial_joint_angle_[0] << "  " << joint_angle_[0] << "  " << ratio);
    //right arm
    result_["r_sho_pitch"]->goal_position_   = initial_joint_angle_[0] + (joint_angle_[0] - initial_joint_angle_[0])*ratio;
    result_["r_sho_roll"]->goal_position_    = initial_joint_angle_[2] + (joint_angle_[2] - initial_joint_angle_[2])*ratio;
    result_["r_el"]->goal_position_          = initial_joint_angle_[4] + (joint_angle_[4] - initial_joint_angle_[4])*ratio;

    // left arm
    result_["l_sho_pitch"]->goal_position_   = initial_joint_angle_[1] + (joint_angle_[1] - initial_joint_angle_[1])*ratio;
    result_["l_sho_roll"]->goal_position_    = initial_joint_angle_[3] + (joint_angle_[3] - initial_joint_angle_[3])*ratio;
    result_["l_el"]->goal_position_          = initial_joint_angle_[5] + (joint_angle_[5] - initial_joint_angle_[5])*ratio;

    // right leg
    result_["r_hip_yaw"]->goal_position_     = initial_joint_angle_[6] + (joint_angle_[6] - initial_joint_angle_[6])*ratio;
    result_["r_hip_roll"]->goal_position_    = initial_joint_angle_[8] + (joint_angle_[8] - initial_joint_angle_[8])*ratio;
    result_["r_hip_pitch"]->goal_position_   = initial_joint_angle_[10] + (joint_angle_[10] - initial_joint_angle_[10])*ratio;
    result_["r_knee"]->goal_position_        = initial_joint_angle_[12] + (joint_angle_[12] - initial_joint_angle_[12])*ratio;
    result_["r_ank_pitch"]->goal_position_   = initial_joint_angle_[14] + (joint_angle_[14] - initial_joint_angle_[14])*ratio;
    result_["r_ank_roll"]->goal_position_    = initial_joint_angle_[16] + (joint_angle_[16] - initial_joint_angle_[16])*ratio;

    // left leg
    result_["l_hip_yaw"]->goal_position_     = initial_joint_angle_[7] + (joint_angle_[7] - initial_joint_angle_[7])*ratio;
    result_["l_hip_roll"]->goal_position_    = initial_joint_angle_[9] + (joint_angle_[9] - initial_joint_angle_[9])*ratio;
    result_["l_hip_pitch"]->goal_position_   = initial_joint_angle_[11] + (joint_angle_[11] - initial_joint_angle_[11])*ratio;
    result_["l_knee"]->goal_position_        = initial_joint_angle_[13] + (joint_angle_[13] - initial_joint_angle_[13])*ratio;
    result_["l_ank_pitch"]->goal_position_   = initial_joint_angle_[15] + (joint_angle_[15] - initial_joint_angle_[15])*ratio;
    result_["l_ank_roll"]->goal_position_    = initial_joint_angle_[17] + (joint_angle_[17] - initial_joint_angle_[17])*ratio;

    // head
    result_["head_pan"]->goal_position_      = initial_joint_angle_[18] + (joint_angle_[18] - initial_joint_angle_[18])*ratio;
    result_["head_tilt"]->goal_position_     = initial_joint_angle_[19] + (joint_angle_[19] - initial_joint_angle_[19])*ratio;

    trajectory_idx_++;
    if (trajectory_idx_ > 375)
      init_flag_ = false;
  }

  if (trajectory_flag_)
  {
    getLiftingTrajectoryAngle(trajectory_idx_);
    //right arm
    result_["r_sho_pitch"]->goal_position_   = joint_angle_[0];
    result_["r_sho_roll"]->goal_position_    = joint_angle_[2];
    result_["r_el"]->goal_position_          = joint_angle_[4];

    // left arm
    result_["l_sho_pitch"]->goal_position_   = joint_angle_[1];
    result_["l_sho_roll"]->goal_position_    = joint_angle_[3];
    result_["l_el"]->goal_position_          = joint_angle_[5];   

    // right leg
    result_["r_hip_yaw"]->goal_position_     = joint_angle_[6];
    result_["r_hip_roll"]->goal_position_    = joint_angle_[8];
    result_["r_hip_pitch"]->goal_position_   = joint_angle_[10];
    result_["r_knee"]->goal_position_        = joint_angle_[12];
    result_["r_ank_pitch"]->goal_position_   = joint_angle_[14];
    result_["r_ank_roll"]->goal_position_    = joint_angle_[16];

    // left leg
    result_["l_hip_yaw"]->goal_position_     = joint_angle_[7];
    result_["l_hip_roll"]->goal_position_    = joint_angle_[9];
    result_["l_hip_pitch"]->goal_position_   = joint_angle_[11];
    result_["l_knee"]->goal_position_        = joint_angle_[13]; 
    result_["l_ank_pitch"]->goal_position_   = joint_angle_[15];
    result_["l_ank_roll"]->goal_position_    = joint_angle_[17];

    // head
    result_["head_pan"]->goal_position_      = joint_angle_[18];
    result_["head_tilt"]->goal_position_     = joint_angle_[19];
    trajectory_idx_++;
    //trajectory_idx_++;
    //ROS_INFO_STREAM (trajectory_idx_ << " " << trajectory_size_);
    if (trajectory_idx_ >= trajectory_size_)
      trajectory_flag_ = false;
  }
}

void LiftingModule::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  // Subscriber
  ros::Subscriber lifting_trajectory_sub = ros_node.subscribe("/adol/lifting/trajectory", 5,
                                                              &LiftingModule::getLiftingTrajectoryCallback, this);
  ros::Subscriber lifting_command_sub = ros_node.subscribe("/adol/lifting/command", 5,
                                                              &LiftingModule::getLiftingCommandCallback, this);

  ros::WallDuration duration(control_cycle_sec_ * 0.5);
  while (ros_node.ok())
      callback_queue.callAvailable(duration);
}

void LiftingModule::getLiftingTrajectoryCallback(const lifting_trajectory_msgs::LiftingTrajectory::ConstPtr &msg)
{
  lifting_trajectory_.right_shoulder_pitch.clear();
  lifting_trajectory_.right_shoulder_roll.clear();
  lifting_trajectory_.right_elbow.clear();
  lifting_trajectory_.left_shoulder_pitch.clear();
  lifting_trajectory_.left_shoulder_roll.clear();
  lifting_trajectory_.left_elbow.clear();
  lifting_trajectory_.head_pitch.clear();
  lifting_trajectory_.head_yaw.clear();
  lifting_trajectory_.pelvis.clear();
  lifting_trajectory_.right_foot.clear();
  lifting_trajectory_.left_foot.clear();

  ROS_INFO_STREAM(msg->right_shoulder_pitch.size());
  for (size_t msg_idx = 0; msg_idx < msg->right_shoulder_pitch.size(); msg_idx++)
  {
    lifting_trajectory_.right_shoulder_pitch.push_back(msg->right_shoulder_pitch[msg_idx]);
    lifting_trajectory_.right_shoulder_roll.push_back(msg->right_shoulder_roll[msg_idx]);
    lifting_trajectory_.right_elbow.push_back(msg->right_elbow[msg_idx]);
    lifting_trajectory_.left_shoulder_pitch.push_back(msg->left_shoulder_pitch[msg_idx]);
    lifting_trajectory_.left_shoulder_roll.push_back(msg->left_shoulder_roll[msg_idx]);
    lifting_trajectory_.left_elbow.push_back(msg->left_elbow[msg_idx]);
    lifting_trajectory_.head_pitch.push_back(msg->head_pitch[msg_idx]);
    lifting_trajectory_.head_yaw.push_back(msg->head_yaw[msg_idx]);
    lifting_trajectory_.pelvis.push_back(msg->pelvis[msg_idx]);
    lifting_trajectory_.right_foot.push_back(msg->right_foot[msg_idx]);
    lifting_trajectory_.left_foot.push_back(msg->left_foot[msg_idx]);
  }
}

void LiftingModule::getLiftingCommandCallback(const std_msgs::String::ConstPtr& msg)
{
  if (msg->data == "play")
  {
    ROS_INFO_STREAM(msg->data);
    trajectory_idx_ = 0;
    trajectory_size_ = lifting_trajectory_.right_shoulder_pitch.size();
    trajectory_flag_ = true;
  }
  else if (msg->data == "init")
  {
    ROS_INFO_STREAM(msg->data);
    trajectory_idx_ = 0;
    trajectory_size_ = 375;
    
    //right arm
    initial_joint_angle_[0] = result_["r_sho_pitch"]->goal_position_;
    initial_joint_angle_[2] = result_["r_sho_roll"]->goal_position_;
    initial_joint_angle_[4] = result_["r_el"]->goal_position_; 

    // left arm
    initial_joint_angle_[1] = result_["l_sho_pitch"]->goal_position_;
    initial_joint_angle_[3] = result_["l_sho_roll"]->goal_position_;
    initial_joint_angle_[5] = result_["l_el"]->goal_position_;        

    // right leg
    initial_joint_angle_[6]  = result_["r_hip_yaw"]->goal_position_;
    initial_joint_angle_[8]  = result_["r_hip_roll"]->goal_position_;
    initial_joint_angle_[10] = result_["r_hip_pitch"]->goal_position_;
    initial_joint_angle_[12] = result_["r_knee"]->goal_position_;
    initial_joint_angle_[14] = result_["r_ank_pitch"]->goal_position_; 
    initial_joint_angle_[16] = result_["r_ank_roll"]->goal_position_;

    // left leg
    initial_joint_angle_[7]  = result_["l_hip_yaw"]->goal_position_;   
    initial_joint_angle_[9]  = result_["l_hip_roll"]->goal_position_;  
    initial_joint_angle_[11] = result_["l_hip_pitch"]->goal_position_; 
    initial_joint_angle_[13] = result_["l_knee"]->goal_position_;      
    initial_joint_angle_[15] = result_["l_ank_pitch"]->goal_position_; 
    initial_joint_angle_[17] = result_["l_ank_roll"]->goal_position_;  

    // head
    initial_joint_angle_[18] = result_["head_pan"]->goal_position_;  
    initial_joint_angle_[19] = result_["head_tilt"]->goal_position_; 

    getLiftingTrajectoryAngle(0);
    init_flag_ = true;
  }
}

void LiftingModule::getLiftingTrajectoryAngle(int trajectory_idx)
{
  if (lifting_trajectory_.right_shoulder_pitch.size() == 0)
    return;
  
  double leg_angle[6];

  //right arm
  joint_angle_[0] = lifting_trajectory_.right_shoulder_pitch[trajectory_idx];
  joint_angle_[2] = lifting_trajectory_.right_shoulder_roll[trajectory_idx];
  joint_angle_[4] = lifting_trajectory_.right_elbow[trajectory_idx];

  //left arms
  joint_angle_[1] = lifting_trajectory_.left_shoulder_pitch[trajectory_idx];
  joint_angle_[3] = lifting_trajectory_.left_shoulder_roll[trajectory_idx];
  joint_angle_[5] = lifting_trajectory_.left_elbow[trajectory_idx];

  //head
  joint_angle_[18] = lifting_trajectory_.head_yaw[trajectory_idx];
  joint_angle_[19] = lifting_trajectory_.head_pitch[trajectory_idx];


  Eigen::Matrix4d g_to_pelvis 
    = getTransformationXYZQuat(lifting_trajectory_.pelvis[trajectory_idx].position.x,
    lifting_trajectory_.pelvis[trajectory_idx].position.y,
    lifting_trajectory_.pelvis[trajectory_idx].position.z,
    lifting_trajectory_.pelvis[trajectory_idx].orientation.w,
    lifting_trajectory_.pelvis[trajectory_idx].orientation.x,
    lifting_trajectory_.pelvis[trajectory_idx].orientation.y,
    lifting_trajectory_.pelvis[trajectory_idx].orientation.z);

  Eigen::Matrix4d pelvis_to_g = robotis_framework::getInverseTransformation(g_to_pelvis);

  Eigen::Matrix4d g_to_rfoot
    = getTransformationXYZQuat(lifting_trajectory_.right_foot[trajectory_idx].position.x,
    lifting_trajectory_.right_foot[trajectory_idx].position.y,
    lifting_trajectory_.right_foot[trajectory_idx].position.z,
    lifting_trajectory_.right_foot[trajectory_idx].orientation.w,
    lifting_trajectory_.right_foot[trajectory_idx].orientation.x,
    lifting_trajectory_.right_foot[trajectory_idx].orientation.y,
    lifting_trajectory_.right_foot[trajectory_idx].orientation.z);

  Eigen::Matrix4d g_to_lfoot
    = getTransformationXYZQuat(lifting_trajectory_.left_foot[trajectory_idx].position.x,
    lifting_trajectory_.left_foot[trajectory_idx].position.y,
    lifting_trajectory_.left_foot[trajectory_idx].position.z,
    lifting_trajectory_.left_foot[trajectory_idx].orientation.w,
    lifting_trajectory_.left_foot[trajectory_idx].orientation.x,
    lifting_trajectory_.left_foot[trajectory_idx].orientation.y,
    lifting_trajectory_.left_foot[trajectory_idx].orientation.z);

  Eigen::Matrix4d rhip_to_rfoot = (mat_rhip_to_pelvis*pelvis_to_g)*g_to_rfoot;
  Eigen::Matrix4d lhip_to_lfoot = (mat_lhip_to_pelvis*pelvis_to_g)*g_to_lfoot;
  
  computeRLegInverseKinematics(leg_angle, rhip_to_rfoot);

  joint_angle_[6] = leg_angle[0];
  joint_angle_[8] = leg_angle[1];
  joint_angle_[10] = leg_angle[2];
  joint_angle_[12] = leg_angle[3];
  joint_angle_[14] = leg_angle[4];
  joint_angle_[16] = leg_angle[5];


  computeLLegInverseKinematics(leg_angle, lhip_to_lfoot);

  joint_angle_[7] = leg_angle[0];
  joint_angle_[9] = leg_angle[1];
  joint_angle_[11] = leg_angle[2];
  joint_angle_[13] = leg_angle[3];
  joint_angle_[15] = leg_angle[4];
  joint_angle_[17] = leg_angle[5];

  // ROS_INFO_STREAM(joint_angle_[0]);
  // ROS_INFO_STREAM(joint_angle_[1]);
  // ROS_INFO_STREAM(joint_angle_[2]);
  // ROS_INFO_STREAM(joint_angle_[3]);
  // ROS_INFO_STREAM(joint_angle_[4]);
  // ROS_INFO_STREAM(joint_angle_[5]);
  // ROS_INFO_STREAM(joint_angle_[6]);
  // ROS_INFO_STREAM(joint_angle_[7]);
  // ROS_INFO_STREAM(joint_angle_[8]);
  // ROS_INFO_STREAM(joint_angle_[9]);
  // ROS_INFO_STREAM(joint_angle_[10]);
  // ROS_INFO_STREAM(joint_angle_[11]);
  // ROS_INFO_STREAM(joint_angle_[12]);
  // ROS_INFO_STREAM(joint_angle_[13]);
  // ROS_INFO_STREAM(joint_angle_[14]);
  // ROS_INFO_STREAM(joint_angle_[15]);
  // ROS_INFO_STREAM(joint_angle_[16]);
  // ROS_INFO_STREAM(joint_angle_[17]);
  // ROS_INFO_STREAM(joint_angle_[18]);
  // ROS_INFO_STREAM(joint_angle_[19]);
}

void LiftingModule::onModuleEnable()
{
  lifting_module_enabled_ = true;
}

void LiftingModule::onModuleDisable()
{
  lifting_module_enabled_ = false;
}

void LiftingModule::stop()
{
  trajectory_flag_ = false;
  init_flag_ = false;
}

bool LiftingModule::isRunning()
{
  return trajectory_flag_ || init_flag_;
}

bool LiftingModule::computeLegInverseKinematics(double *angle, Eigen::Matrix4d& transform)
{
  double out[6];
  double hip_pitch_offset_m_ = 0.0001;
  double hip_offset_angle_rad_ = atan2(0.0001, 0.11015);
  double thigh_length_m_ = sqrt(0.0001*0.0001 + 0.11015*0.11015);
  double calf_length_m_ = 0.11;
  double ankle_length_m_ = 0.0305;

  Eigen::Matrix3d R06 = transform.block<3,3>(0,0);
  Eigen::Vector3d p06 = transform.block<3,1>(0,3) + ankle_length_m_*R06.block<3,1>(0,2); //desired hip to ankle
    
  //calc q6
  Eigen::Vector3d p60 = -R06.transpose()*p06;
  *(out + 5) = atan2(p60(1), p60(2));

  //calc q1
  Eigen::Matrix3d R05 = R06*robotis_framework::getRotationX(-(*(out + 5)));
  *(out + 0) = atan2(-R05(0, 1), R05(1, 1));

  //calc q4
  Eigen::Vector3d p03 = robotis_framework::getRotationZ(*(out + 0))*robotis_framework::getTransitionXYZ(hip_pitch_offset_m_, 0, 0);
  Eigen::Vector3d p36 = p06 - p03;
  
  *(out + 3) = -acos((thigh_length_m_*thigh_length_m_ + calf_length_m_*calf_length_m_ - p36.norm()*p36.norm())/(2*thigh_length_m_*calf_length_m_)) + EIGEN_PI;

  //calc q5
  double alpha = asin(thigh_length_m_*sin(EIGEN_PI - *(out + 3))/p36.norm());
  Eigen::Vector3d p63 = -R06.transpose()*p36;
  *(out + 4) = -atan2(p63(0), getSign(p63(2))*sqrt(p63(1)*p63(1) + p63(2)*p63(2))) - alpha;

  //calc q2 and q3
  Eigen::Matrix3d R13 = robotis_framework::getRotationZ(-(*(out + 0))) * R05 * robotis_framework::getRotationY( -(*(out + 4) + *(out + 3)) );
  *(out + 1) = atan2(R13(2,1), R13(1,1));
  *(out + 2) = atan2(R13(0,2), R13(0,0));
  
  *(out + 2) += hip_offset_angle_rad_;
  *(out + 3) += -hip_offset_angle_rad_;

  angle[0] = out[0];
  angle[1] = out[1];
  angle[2] = out[2];
  angle[3] = out[3];
  angle[4] = out[4];
  angle[5] = out[5];

  return true;
}

bool LiftingModule::computeRLegInverseKinematics(double *angle, Eigen::Matrix4d& transform)
{
  computeLegInverseKinematics(angle, transform);

  angle[0] = -angle[0];
  angle[1] = -angle[1];
  angle[2] = -angle[2];
  angle[3] = -angle[3];
  angle[4] =  angle[4];
  angle[5] =  angle[5];

  return true;
}

bool LiftingModule::computeLLegInverseKinematics(double *angle, Eigen::Matrix4d& transform)
{
  computeLegInverseKinematics(angle, transform);

  angle[0] = -angle[0];
  angle[1] = -angle[1];
  angle[2] =  angle[2];
  angle[3] =  angle[3];
  angle[4] = -angle[4];
  angle[5] =  angle[5];

  return true;
}