#include <ros/ros.h>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

#include <robotis_math/robotis_math.h>

#include <std_msgs/String.h>
#include <lifting_trajectory_msgs/LiftingTrajectory.h>



void initializeMessage();
bool loadTrajectory(std::string& file_path);

lifting_trajectory_msgs::LiftingTrajectory lifting_msg;
ros::Publisher lifting_msg_pub_, lifting_cmd_pub_;
ros::Publisher set_ctrl_module_pub_;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lifting_trajectory_loader");
  ros::NodeHandle nh;

  lifting_msg_pub_ = nh.advertise<lifting_trajectory_msgs::LiftingTrajectory>("/adol/lifting/trajectory", 0);
  lifting_cmd_pub_ = nh.advertise<std_msgs::String>("/adol/lifting/command", 0);
  set_ctrl_module_pub_ = nh.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 1);

  std::string cmd, file_path, basic_path;
  basic_path = "/home/robotis/lifting_trajectory/";

  while(ros::ok())
  {
    std::cout << "[CMD]: " << std::endl;
    std::cin >> cmd;

    if (cmd == "exit")
      break;
    else if (cmd == "send") 
    {
      if (lifting_msg.right_shoulder_pitch.size() != 0)
        lifting_msg_pub_.publish(lifting_msg);
      else
        std::cout << "there is no trajectory to send" << std::endl;
    }
    else if (cmd == "play")
    {
      std_msgs::String msg;
      msg.data = "play";
      lifting_cmd_pub_.publish(msg);
    }
    else if (cmd == "init")
    {
      std_msgs::String msg;
      msg.data = "init";
      lifting_cmd_pub_.publish(msg);
    }
    else if (cmd == "set")
    {
      std_msgs::String control_msg;
      control_msg.data = "lifting_module";

      set_ctrl_module_pub_.publish(control_msg);
    }
    else
    {
      file_path = basic_path + cmd + ".txt";
      if (loadTrajectory(file_path) == false)
        std::cout << "Failed to open the file: " << file_path << std::endl;
      else
        std::cout << "Succeeded in reading the file: " << file_path << std::endl;
    }
  }

  return 0;
}

bool loadTrajectory(std::string& file_path)
{
  std::ifstream file(file_path.c_str());

  if (file.is_open() == false)
    return false;

  initializeMessage();

  std::stringstream ss;
  std::string line_buffer;

  double arm_angle_rad[6];
  robotis_framework::Pose3D pelvis, right_foot, left_foot;
  geometry_msgs::Pose pelvis_msg, right_foot_msg, left_foot_msg;

  while (std::getline(file, line_buffer))
  {
    ss.str(line_buffer);

    ss >> arm_angle_rad[0] >> arm_angle_rad[1] >> arm_angle_rad[2] 
    >> arm_angle_rad[3] >> arm_angle_rad[4] >> arm_angle_rad[5] 
    >> pelvis.x >> pelvis.y >> pelvis.z >> pelvis.roll >> pelvis.pitch >> pelvis.yaw
    >> right_foot.x >> right_foot.y >> right_foot.z >> right_foot.roll >> right_foot.pitch >> right_foot.yaw
    >> left_foot.x  >> left_foot.y  >> left_foot.z  >> left_foot.roll  >> left_foot.pitch  >> left_foot.yaw;

    lifting_msg.right_shoulder_pitch.push_back(arm_angle_rad[0]);
    lifting_msg.left_shoulder_pitch.push_back(arm_angle_rad[1]);
    lifting_msg.right_shoulder_roll.push_back(arm_angle_rad[2]);
    lifting_msg.left_shoulder_roll.push_back(arm_angle_rad[3]);
    lifting_msg.right_elbow.push_back(arm_angle_rad[4]);
    lifting_msg.left_elbow.push_back(arm_angle_rad[5]);

    lifting_msg.head_yaw.push_back(0);
    lifting_msg.head_pitch.push_back(0);
    
    Eigen::Quaterniond quat;
    quat = robotis_framework::convertRPYToQuaternion(pelvis.roll, pelvis.pitch, pelvis.yaw);
    pelvis_msg.position.x = pelvis.x;
    pelvis_msg.position.y = pelvis.y;
    pelvis_msg.position.z = pelvis.z;
    pelvis_msg.orientation.x = quat.x();
    pelvis_msg.orientation.y = quat.y();
    pelvis_msg.orientation.z = quat.z();
    pelvis_msg.orientation.w = quat.w();
    lifting_msg.pelvis.push_back(pelvis_msg);

    quat = robotis_framework::convertRPYToQuaternion(right_foot.roll, right_foot.pitch, right_foot.yaw);
    right_foot_msg.position.x = right_foot.x;
    right_foot_msg.position.y = right_foot.y;
    right_foot_msg.position.z = right_foot.z;
    right_foot_msg.orientation.x = quat.x();
    right_foot_msg.orientation.y = quat.y();
    right_foot_msg.orientation.z = quat.z();
    right_foot_msg.orientation.w = quat.w();
    lifting_msg.right_foot.push_back(right_foot_msg);

    quat = robotis_framework::convertRPYToQuaternion(left_foot.roll, left_foot.pitch, left_foot.yaw);
    left_foot_msg.position.x = left_foot.x;
    left_foot_msg.position.y = left_foot.y;
    left_foot_msg.position.z = left_foot.z;
    left_foot_msg.orientation.x = quat.x();
    left_foot_msg.orientation.y = quat.y();
    left_foot_msg.orientation.z = quat.z();
    left_foot_msg.orientation.w = quat.w();
    lifting_msg.left_foot.push_back(left_foot_msg);

    // std::cout << arm_angle_rad[0] << " "<<  arm_angle_rad[1] << " "<< arm_angle_rad[2] 
    // << " "<< arm_angle_rad[3] << " "<<  arm_angle_rad[4] << " "<<  arm_angle_rad[5] 
    // << "/ "<< pelvis.x << " " << pelvis.y << " "<<  pelvis.z << " "<<  pelvis.roll << " " << pelvis.pitch << " " <<  pelvis.yaw
    // << "/ "<< right_foot.x << " " << right_foot.y << " " << right_foot.z << " "<< right_foot.roll << " "<< right_foot.pitch << " "<< right_foot.yaw
    // << "/ "<< left_foot.x  << " " << left_foot.y  << " " << left_foot.z  << " "<< left_foot.roll  << " "<< left_foot.pitch  << " "<< left_foot.yaw;
    // std::cout << std::endl;
    
    ss.str(""); //to clear
    ss.clear();
  }

  file.close();

  return true;
}

void initializeMessage()
{
  lifting_msg.right_shoulder_pitch.clear();
  lifting_msg.right_shoulder_roll.clear();
  lifting_msg.right_elbow.clear();

  lifting_msg.left_shoulder_pitch.clear();
  lifting_msg.left_shoulder_roll.clear();
  lifting_msg.left_elbow.clear();

  lifting_msg.head_pitch.clear();
  lifting_msg.head_yaw.clear();

  lifting_msg.pelvis.clear();
  lifting_msg.right_foot.clear();
  lifting_msg.left_foot.clear();
}