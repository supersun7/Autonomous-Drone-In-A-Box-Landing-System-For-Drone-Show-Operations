#include "landing_quadrotor.h"
using namespace std;
using namespace Eigen;

PX4Landing::PX4Landing(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
: nh_(nh), nh_private_(nh_private) {
  Initialize();
  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.1), &PX4Landing::CmdLoopCallback, this);

  ar_pose_sub_ = nh_private_.subscribe("/ar_pose_marker", 1, &PX4Landing::ArPoseCallback, this, ros::TransportHints().tcpNoDelay());
  position_sub_ = nh_private_.subscribe("/mavros/local_position/pose", 1, &PX4Landing::Px4PosCallback, this, ros::TransportHints().tcpNoDelay());
  state_sub_    = nh_private_.subscribe("/mavros/state",            1, &PX4Landing::Px4StateCallback, this, ros::TransportHints().tcpNoDelay());
  set_mode_client_ = nh_private_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
}

PX4Landing::~PX4Landing(){}

inline double clampAbs(double v, double m){ return std::max(-m, std::min(m, v)); }

Eigen::Vector4d PX4Landing::LandingPidProcess(Eigen::Vector3d &currentPos,float currentYaw,Eigen::Vector3d &expectPos,float expectYaw)
{
  Eigen::Vector4d out;

  // X
  s_PidItemX.difference = expectPos[0] - currentPos[0];
  s_PidItemX.intergral += s_PidItemX.difference;
  s_PidItemX.intergral  = clamp(s_PidItemX.intergral, 20.0);
  s_PidItemX.differential = s_PidItemX.difference - s_PidItemX.tempDiffer;
  s_PidItemX.tempDiffer = s_PidItemX.difference;
  out[0] = s_PidXY.p*s_PidItemX.difference + s_PidXY.d*s_PidItemX.differential + s_PidXY.i*s_PidItemX.intergral;

  // Y
  s_PidItemY.difference = expectPos[1] - currentPos[1];
  s_PidItemY.intergral += s_PidItemY.difference;
  s_PidItemY.intergral  = clamp(s_PidItemY.intergral, 20.0);
  s_PidItemY.differential = s_PidItemY.difference - s_PidItemY.tempDiffer;
  s_PidItemY.tempDiffer = s_PidItemY.difference;
  out[1] = s_PidXY.p*s_PidItemY.difference + s_PidXY.d*s_PidItemY.differential + s_PidXY.i*s_PidItemY.intergral;

  // Z
  s_PidItemZ.difference = expectPos[2] - currentPos[2];
  s_PidItemZ.intergral += s_PidItemZ.difference;
  s_PidItemZ.intergral  = clamp(s_PidItemZ.intergral, 20.0);
  s_PidItemZ.differential = s_PidItemZ.difference - s_PidItemZ.tempDiffer;
  s_PidItemZ.tempDiffer = s_PidItemZ.difference;
  out[2] = s_PidZ.p*s_PidItemZ.difference + s_PidZ.d*s_PidItemZ.differential + s_PidZ.i*s_PidItemZ.intergral;

  // Yaw
  float yaw_err = wrapPi(expectYaw - currentYaw);
  s_PidItemYaw.difference = yaw_err;
  s_PidItemYaw.intergral += s_PidItemYaw.difference;
  s_PidItemYaw.intergral  = clamp(s_PidItemYaw.intergral, 20.0);
  s_PidItemYaw.differential = s_PidItemYaw.difference - s_PidItemYaw.tempDiffer;
  s_PidItemYaw.tempDiffer = s_PidItemYaw.difference;
  out[3] = s_PidYaw.p*s_PidItemYaw.difference + s_PidYaw.d*s_PidItemYaw.differential + s_PidYaw.i*s_PidItemYaw.intergral;

  // 限幅
  out[0] = clampAbs(out[0], 0.5);
  out[1] = clampAbs(out[1], 0.5);
  out[2] = clampAbs(out[2], 0.4);
  out[3] = clampAbs(out[3], 0.6);
  return out;
}

void PX4Landing::CmdLoopCallback(const ros::TimerEvent&){ LandingStateUpdate(); }

void PX4Landing::LandingStateUpdate()
{
  // 终止态：彻底停止
  if (LandingState == DONE || landed_session_done_) return;

  switch(LandingState)
  {
    case WAITING:
      // 不再在 WAITING 自动切 OFFBOARD，避免和 AUTO.LAND 竞争
      if(px4_state_.mode == "OFFBOARD"){
        temp_pos_drone = px4_pose_;
        LandingState = CHECKING;
        cout << "CHECKING" << endl;
      }
      break;

    case CHECKING:
      if(px4_pose_[0] == 0 && px4_pose_[1] == 0){
        mode_cmd_.request.custom_mode = "AUTO.LAND";
        set_mode_client_.call(mode_cmd_);
        LandingState = WAITING;
      }else{
        LandingState = PREPARE;
        cout << "PREPARE" << endl;
      }
      break;

    case PREPARE:
      posxyz_target = temp_pos_drone;
      posxyz_target[2] = search_alt_;
      if(allow_offboard_stream_ && px4_state_.mode=="OFFBOARD"){
        OffboardControl_.send_pos_setpoint(posxyz_target, 0);
      }
      if((px4_pose_[2] <= search_alt_+0.1) && (px4_pose_[2] >= search_alt_-0.1)){
        LandingState = SEARCH;
      }
      if(px4_state_.mode != "OFFBOARD"){ LandingState = WAITING; }
      break;

    case SEARCH:
      if(detect_state){
        resetPid();
        stable_cnt_ = 0;
        LandingState = LANDING;
        cout << "LANDING" << endl;
      }else{
        if(allow_offboard_stream_ && px4_state_.mode=="OFFBOARD"){
          OffboardControl_.send_pos_setpoint(posxyz_target, 0);
        }
      }
      if(px4_state_.mode != "OFFBOARD"){ LandingState = WAITING; }
      break;

    case LANDING:
    {
      double ex = ar_pose_[0] - desire_pose_[0];
      double ey = ar_pose_[1] - desire_pose_[1];
      double r  = std::sqrt(ex*ex + ey*ey);

      if(detect_state){
        desire_vel_ = LandingPidProcess(ar_pose_, markers_yaw_, desire_pose_, desire_yaw_);
        if(r > 0.15) { desire_vel_[3] = 0.0; } // 未居中先不转头，避免盘旋
      }else{
        desire_vel_[0]=desire_vel_[1]=0.0;
        desire_vel_[2]=-0.2;
        desire_vel_[3]=0.0;
      }

      double vxy = std::sqrt(desire_vel_[0]*desire_vel_[0] + desire_vel_[1]*desire_vel_[1]);

      // 更稳的触发：低高度 + 居中 + 速度小，连续 0.3s
      if (ar_pose_[2] <= 0.5 && r < 0.12 && vxy < 0.08) stable_cnt_++; else stable_cnt_ = 0;
      if (stable_cnt_ >= 3) {
        LandingState = FINAL_BRAKE;
        cout << "FINAL_BRAKE" << endl;
      }

      if(allow_offboard_stream_ && px4_state_.mode=="OFFBOARD"){
        desire_xyVel_[0] = desire_vel_[0];
        desire_xyVel_[1] = desire_vel_[1];
        desire_xyVel_[2] = desire_vel_[2];
        desire_yawVel_   = desire_vel_[3];
        OffboardControl_.send_body_velxyz_setpoint(desire_xyVel_, desire_yawVel_);
      }

      if(px4_state_.mode != "OFFBOARD"){ LandingState = WAITING; }
    }
    break;

    case FINAL_BRAKE:
    {
      // 原地刹车 1s
      for (int i=0;i<10;++i){
        Eigen::Vector3d zero_v(0,0,0);
        OffboardControl_.send_body_velxyz_setpoint(zero_v, 0.0f);
        ros::Duration(0.1).sleep();
      }
      resetPid();
      LandingState = LANDOVER;
      cout << "LANDOVER" << endl;
    }
    break;

    case LANDOVER:
    {
      // **关键改动：进入 LANDOVER 立刻终止本节点的控制能力**
      landed_session_done_ = true;          // 立刻置位，防止下一拍再进状态机
      allow_offboard_stream_ = false;       // 停止任何 setpoint
      resetPid();

      // 切 AUTO.LAND（若已在 LAND 也无妨）
      mode_cmd_.request.custom_mode = "AUTO.LAND";
      set_mode_client_.call(mode_cmd_);

      // 停止定时器，彻底不再调度状态机
      cmdloop_timer_.stop();

      // （可选）延时给飞控收尾
      ros::Duration(2.0).sleep();

      // 切入终止态
      LandingState = DONE;
      ROS_INFO("[landing] Landing session finished. Controller disabled.");
    }
    break;

    case DONE:
    default:
      // 什么都不做
      break;
  }
}

/* 接收降落板相对飞机的位置以及偏航角 */
void PX4Landing::ArPoseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
{
  detect_state = false;
  double rr,pp,yy;
  tf::Quaternion quat;
  for(auto &item : msg->markers){
    if(item.id == markers_id_){
      detect_state = true;
      ar_pose_[0] = -item.pose.pose.position.x;
      ar_pose_[1] =  item.pose.pose.position.y;
      ar_pose_[2] =  item.pose.pose.position.z;
      tf::quaternionMsgToTF(item.pose.pose.orientation, quat);
      tf::Matrix3x3(quat).getRPY(rr,pp,yy);
      markers_yaw_ = yy;
    }
  }
}

void PX4Landing::Px4PosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  px4_pose_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

void PX4Landing::Px4StateCallback(const mavros_msgs::State::ConstPtr& msg)
{
  px4_state_ = *msg;
}

void PX4Landing::Initialize()
{
  nh_private_.param<float>("search_alt_", search_alt_, 3.f);

  int mid = 4;
  nh_private_.param<int>("markers_id_", mid, 4);
  markers_id_ = mid;

  nh_private_.param<float>("PidXY_p",  s_PidXY.p, 0.4f);
  nh_private_.param<float>("PidXY_d",  s_PidXY.d, 0.05f);
  nh_private_.param<float>("PidXY_i",  s_PidXY.i, 0.01f);
  nh_private_.param<float>("PidZ_p",   s_PidZ.p,  0.1f);
  nh_private_.param<float>("PidZ_d",   s_PidZ.d,  0.0f);
  nh_private_.param<float>("PidZ_i",   s_PidZ.i,  0.0f);
  nh_private_.param<float>("PidYaw_p", s_PidYaw.p,0.0f);
  nh_private_.param<float>("PidYaw_d", s_PidYaw.d,0.0f);
  nh_private_.param<float>("PidYaw_i", s_PidYaw.i,0.0f);

  float dx,dy,dz;
  nh_private_.param<float>("desire_pose_x", dx, 0.f);
  nh_private_.param<float>("desire_pose_y", dy, 0.f);
  nh_private_.param<float>("desire_pose_z", dz, 0.f);
  nh_private_.param<float>("desire_yaw_",   desire_yaw_, 0.f);
  desire_pose_ << dx, dy, dz;

  detect_state = false;
  allow_offboard_stream_ = true;
  landed_session_done_ = false;
  stable_cnt_ = 0;

  desire_vel_.setZero();
  desire_xyVel_.setZero();
  desire_yawVel_ = 0.f;

  resetPid();
}

void PX4Landing::resetPid(){
  s_PidItemX.intergral = s_PidItemY.intergral = s_PidItemZ.intergral = s_PidItemYaw.intergral = 0;
  s_PidItemX.tempDiffer = s_PidItemY.tempDiffer = s_PidItemZ.tempDiffer = s_PidItemYaw.tempDiffer = 0;
}

int main(int argc, char** argv)
{
  ros::init(argc,argv,"landing_quadrotor");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");
  PX4Landing node(nh, nh_private);
  ros::spin();
  return 0;
}
