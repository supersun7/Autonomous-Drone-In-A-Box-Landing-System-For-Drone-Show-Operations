#pragma once
#include <ros/ros.h>
#include <iostream>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt32.h>
#include "offboard_control.h"
#include "px4_control_cfg.h"
#include <tf/transform_datatypes.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

using namespace std;
using namespace Eigen;

class PX4Landing {
 public:
  PX4Landing(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  ~PX4Landing();

  void Initialize();
  OffboardControl OffboardControl_;

 private:
  // 工具函数
  inline float wrapPi(float a){
    while(a >  M_PI) a -= 2.f*M_PI;
    while(a < -M_PI) a += 2.f*M_PI;
    return a;
  }
  inline double clamp(double v, double m){
    return std::max(-m, std::min(m, v));
  }
  void resetPid();

  // 回调/流程
  void CmdLoopCallback(const ros::TimerEvent& event);
  void LandingStateUpdate();
  void ArPoseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg);
  void Px4PosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void Px4StateCallback(const mavros_msgs::State::ConstPtr& msg);
  Eigen::Vector4d LandingPidProcess(Eigen::Vector3d &currentPos,float currentYaw,Eigen::Vector3d &expectPos,float expectYaw);

  // 变量
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Timer cmdloop_timer_;

  Eigen::Vector3d temp_pos_drone;
  Eigen::Vector3d posxyz_target;
  Eigen::Vector3d velxy_posz_target;
  Eigen::Vector3d ar_pose_;
  Eigen::Vector3d px4_pose_;
  Eigen::Vector3d desire_pose_;
  float desire_yaw_;
  float markers_yaw_;
  float search_alt_;
  int   markers_id_;

  mavros_msgs::State px4_state_;
  mavros_msgs::SetMode mode_cmd_;

  bool detect_state;
  bool allow_offboard_stream_;
  bool landed_session_done_;   // 一次降落完成后彻底停机
  int  stable_cnt_;

  Eigen::Vector4d desire_vel_;
  Eigen::Vector3d desire_xyVel_;
  float desire_yawVel_;

  S_PID s_PidXY,s_PidZ,s_PidYaw;
  S_PID_ITEM s_PidItemX;
  S_Pid_ITEM s_PidItemY;
  S_PID_ITEM s_PidItemZ;
  S_PID_ITEM s_PidItemYaw;

  enum {
    WAITING,
    CHECKING,
    PREPARE,
    SEARCH,
    LANDING,
    FINAL_BRAKE,
    LANDOVER,
    DONE                 // 终止态：完全不再控制
  } LandingState = WAITING;

  ros::Subscriber ar_pose_sub_;
  ros::Subscriber position_sub_;
  ros::Subscriber state_sub_;
  ros::ServiceClient set_mode_client_;
};
