#include <algorithm>
#include <vector>
#include <string>
#include <deque>
#include <queue>
#include <cmath>

#include <ros/ros.h>

#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Order.h>
#include <nist_gear/Proximity.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#include <std_srvs/Trigger.h>

#include <nist_gear/VacuumGripperControl.h>
#include <nist_gear/ConveyorBeltState.h>
#include <nist_gear/VacuumGripperState.h>
#include <nist_gear/AGVControl.h>

#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <trajectory_msgs/JointTrajectory.h>

#include <thread>

#define PI_2 6.283185307179586
#define PI 3.141592653589793


#define MAIN_FILE
using namespace std;

int inverse(const double* T, double* q_sols, double q6_des, bool lin = false);


ros::Time start_stamp;

enum PType{
  EMPTY = 0,
  GASKET_R = 1,
  GASKET_G = 2,
  GASKET_B = 3,
  GEAR_R = 4,
  GEAR_G = 5,
  GEAR_B = 6,
  PISTON_ROD_R = 7,
  PISTON_ROD_G = 8,
  PISTON_ROD_B = 9,
  PULLEY_R = 10,
  PULLEY_G = 11,
  PULLEY_B = 12
};

enum State{
  IDLE = 0,
  ONE = 1,
  TWO = 2
  // BELT=1,
  // TRANSFER=2,
  // TRANSIT=3,
  // FAULTY=4,
  // FLIP=5,
  // INV=6,
};
class Shipment{
public:
  int priority;
  int agv;
  int c;
  vector<pair<double,double>> position;
  vector<double> theta;
  vector<int> obj_t;
  vector<bool> flipped;
  vector<bool> finished;
  vector<bool> finished_2;
  vector<bool> invalid;
  string shipment_t;
  Shipment(int priority=100, int c=0):priority(priority), c(c){
  }
};
struct part_bin_pose{
  double x, y, theta;
  bool flipped;
  part_bin_pose(double x, double y, double theta, bool flipped):x(x), y(y), theta(theta), flipped(flipped){}
};

struct part_belt_pose{
  ros::Time st;
  int type;
  double theta;
  bool flipped;
  part_belt_pose(ros::Time start, int type, double theta, bool flippped):st(start), type(type), theta(theta), flipped(flipped){}
};

inline int type2int(const string s){
  int res;
  if(s[1] == 'a') res = 1;
  else if(s[1] == 'e') res = 4;
  else if(s[1] == 'i') res =7;
  else res =10;
  // red green blue
  if(s.back()=='n') res++;
  else if(s.back()=='e') res+=2;
  return res;
};

void start_competition(ros::NodeHandle & node) {
  // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
  ros::ServiceClient start_client =
    node.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  // If it's not already ready, wait for it to be ready.
  // Calling the Service using the client before the server is ready would fail.
  if (!start_client.exists()) {
    ROS_INFO("Waiting for the competition to be ready...");
    start_client.waitForExistence();
    ROS_INFO("Competition is now ready.");
  }
  ROS_INFO("Requesting competition start...");
  std_srvs::Trigger srv;  // Combination of the "request" and the "response".
  start_client.call(srv);  // Call the start Service.
  if (!srv.response.success) {  // If not successful, print out why.
    ROS_ERROR_STREAM("Failed to start the competition: " << srv.response.message);
  } else {
    ROS_INFO("Competition started!");
  }
  start_stamp = ros::Time::now();
}
// %EndTag(START_COMP)%

class MyCompetitionClass
{
public:
  bool arm_1_has_been_zeroed_, arm_2_has_been_zeroed_, gantry_has_been_zeroed_;
  explicit MyCompetitionClass(ros::NodeHandle & node)
  : arm_1_has_been_zeroed_(false), arm_2_has_been_zeroed_(false), gantry_has_been_zeroed_(false)
  {
    // %Tag(ADV_CMD)%
    arm_1_joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>(
      "/ariac/gantry/left_arm_controller/command", 10);

    arm_2_joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>(
      "/ariac/gantry/right_arm_controller/command", 10);
    // %EndTag(ADV_CMD)%

    gantry_joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>(
      "/ariac/gantry/gantry_controller/command", 10);

    arm_1_gripper_ctrl = node.serviceClient<nist_gear::VacuumGripperControl>("/ariac/gantry/left_arm/gripper/control");
    arm_2_gripper_ctrl = node.serviceClient<nist_gear::VacuumGripperControl>("/ariac/gantry/right_arm/gripper/control");

    agv_1 = node.serviceClient<nist_gear::AGVControl>("/ariac/agv1");
    agv_2 = node.serviceClient<nist_gear::AGVControl>("/ariac/agv2");

    double T[12] = {
      1, 0, 0, 0.95,
      0, 1, 0, 0,
      0, 0, 1, 0.1,
    };

    //belt
    for(int i=0;i<4;i++){
      T[3] = 0.95 - part_height[i] + part_height[3];
      inverse(T, q_sol_belt_1[i][0], 0);
      T[3] = 1.09 - part_height[i];
      inverse(T, q_sol_belt_1[i][1], 0);
      inverse(T, q_sol_belt_1[i][2], 0);
      T[3] = 0.8 - part_height[i] + part_height[3];
      inverse(T, q_sol_belt_1[i][3], 0);
    }

    //bin
    for(int i=0;i<4;i++){
      T[3] = 0.95;
      inverse(T, q_sol_bin_1[i][0], 0);
      T[3] = 1.26 - part_height[i];
      inverse(T, q_sol_bin_1[i][1], 0);
      inverse(T, q_sol_bin_1[i][2], 0);
      T[3] = 0.9;
      inverse(T, q_sol_bin_1[i][3], 0);
    }

    T[3] = 0.9;
    inverse(T, q_sol_bin_1_put[0], 0);
    T[3] = 0.95;
    inverse(T, q_sol_bin_1_put[1], 0);
    
    //shelf
    T[7] = 0.9;
    for(int i=0;i<4;i++){
      T[3] = 0.35 - part_height[i] + part_height[3];
      inverse(T, q_sol_shelf_1[0][i][0], 0, true);
      inverse(T, q_sol_shelf_1[0][i][1], 0, true);
      T[3] = 0.57 - part_height[i];
      inverse(T, q_sol_shelf_1[0][i][2], 0, true);
      inverse(T, q_sol_shelf_1[0][i][3], 0, true);
      T[3] = 0.3 - part_height[i] + part_height[3];
      inverse(T, q_sol_shelf_1[0][i][4], 0, true);
      memcpy(q_sol_shelf_1[0][i][5], rest_joints_1, 6 * sizeof(double));
    }
    T[3] = 0.3; inverse(T, q_sol_shelf_1_put[0][0], 0, true);
    T[3] = 0.3; inverse(T, q_sol_shelf_1_put[0][1], 0, true);
    T[3] = 0.35; inverse(T, q_sol_shelf_1_put[0][2], 0, true);


    T[7] = -0.9;
    for(int i=0;i<4;i++){
      T[3] = 0.35 - part_height[i] + part_height[3];
      inverse(T, q_sol_shelf_1[1][i][0], 0, true);
      inverse(T, q_sol_shelf_1[1][i][1], 0, true);
      T[3] = 0.57 - part_height[i];
      inverse(T, q_sol_shelf_1[1][i][2], 0, true);
      inverse(T, q_sol_shelf_1[1][i][3], 0, true);
      T[3] = 0.3 - part_height[i] + part_height[3];
      inverse(T, q_sol_shelf_1[1][i][4], 0, true);
      memcpy(q_sol_shelf_1[1][i][5], rest_joints_1, 6 * sizeof(double));
    }

    T[3] = 0.3; inverse(T, q_sol_shelf_1_put[1][0], 0, true);
    T[3] = 0.3; inverse(T, q_sol_shelf_1_put[1][1], 0, true);
    T[3] = 0.35; inverse(T, q_sol_shelf_1_put[1][2], 0, true);

    // neg x and z for right arm
    T[0] = -1;
    T[7] = 0;
    T[10] = -1;

    // belt
    for(int i=0;i<4;i++){
      T[3] = -0.95 + part_height[i] - part_height[3];
      inverse(T, q_sol_belt_2[i][0], 0);
      T[3] = -1.09 + part_height[i];
      inverse(T, q_sol_belt_2[i][1], 0);
      inverse(T, q_sol_belt_2[i][2], 0);
      T[3] = -0.8 + part_height[i] - part_height[3];
      inverse(T, q_sol_belt_2[i][3], 0);
    }

    // bin
    for(int i=0;i<4;i++){
      T[3] =  -0.95 + part_height[i] - part_height[3];
      inverse(T, q_sol_bin_2[i][0], 0);
      T[3] = -1.26 + part_height[i];
      inverse(T, q_sol_bin_2[i][1], 0);
      inverse(T, q_sol_bin_2[i][2], 0);
      T[3] = -0.9 + part_height[i] - part_height[3];
      inverse(T, q_sol_bin_2[i][3], 0);
    }

    T[3] = -0.9; 
    inverse(T, q_sol_bin_2_put[0], 0);
    T[3] = -0.95;
    inverse(T, q_sol_bin_2_put[1], 0);

    //shelf
    T[7] = 0.9;
    for(int i=0;i<4;i++){
      T[3] = -0.35 + part_height[i] - part_height[3];
      inverse(T, q_sol_shelf_2[0][i][0], 0, true);
      inverse(T, q_sol_shelf_2[0][i][1], 0, true);
      T[3] = -0.57 + part_height[i];
      inverse(T, q_sol_shelf_2[0][i][2], 0, true);
      inverse(T, q_sol_shelf_2[0][i][3], 0, true);
      T[3] = -0.3 + part_height[i] - part_height[3];
      inverse(T, q_sol_shelf_2[0][i][4], 0, true);
      memcpy(q_sol_shelf_2[0][i][5], rest_joints_2, 6 * sizeof(double));
    }
    
    T[3] = -0.3; inverse(T, q_sol_shelf_2_put[0][0], 0, true);
    T[3] = -0.3; inverse(T, q_sol_shelf_2_put[0][1], 0, true);
    T[3] = -0.35; inverse(T, q_sol_shelf_2_put[0][2], 0, true);

    T[7] = -0.9;
    for(int i=0;i<4;i++){
      T[3] = -0.35 + part_height[i] - part_height[3];
      inverse(T, q_sol_shelf_2[1][i][0], 0, true);
      inverse(T, q_sol_shelf_2[1][i][1], 0, true);
      T[3] = -0.57 + part_height[i];
      inverse(T, q_sol_shelf_2[1][i][2], 0, true);
      inverse(T, q_sol_shelf_2[1][i][3], 0), true;
      T[3] = -0.3 + part_height[i] - part_height[3];
      inverse(T, q_sol_shelf_2[1][i][4], 0), true;
      memcpy(q_sol_shelf_2[1][i][5], rest_joints_2, 6 * sizeof(double));
    }

    T[3] = -0.3; inverse(T, q_sol_shelf_2_put[1][0], 0, true);
    T[3] = -0.3; inverse(T, q_sol_shelf_2_put[1][1], 0, true);
    T[3] = -0.35; inverse(T, q_sol_shelf_2_put[1][2], 0, true);

    // for(int i=0;i<6;i++)cout<<q_sol_shelf_1[0][0][0][i]<<' ';cout<<endl;
    // for(int i=0;i<6;i++)cout<<q_sol_shelf_1[0][0][1][i]<<' ';cout<<endl;
  }
  vector<Shipment> shipments_1, shipments_2;
  int bin_t2int(string s){
    if(s[1]=='n') return shipments_1.size()>shipments_2.size()? 2:1;
    if(s[3]=='1') return 1;
    return 2;
  }
  /// Called when a new Order message is received.
  void order_callback(const nist_gear::Order::ConstPtr & order_msg) {
    for(const auto & item: order_msg->shipments){
      Shipment *tmp;
      for(int i=0;i<shipments_1.size();i++){
        if(shipments_1[i].shipment_t == item.shipment_type){
          // cout<<"UPdating"<<endl;
          fill(shipments_1[i].invalid.begin(),shipments_1[i].invalid.end(),true);
        }
      }
      for(int i=0;i<shipments_2.size();i++){
        if(shipments_2[i].shipment_t == item.shipment_type){
          fill(shipments_2[i].invalid.begin(),shipments_2[i].invalid.end(),true);
        }
      }
      int n = bin_t2int(item.agv_id);
      if(n==1) {
        shipments_1.resize(shipments_1.size()+1);
        tmp = &shipments_1[shipments_1.size()-1];
        tmp->agv = 1;
      }
      else if(n==2){
        shipments_2.resize(shipments_2.size()+1);
        tmp = &shipments_2[shipments_2.size()-1];
        tmp->agv = 2;
      }
      tmp->shipment_t=(item.shipment_type);
      
      for(auto & item1: item.products){
        // cout<<item1.type<<" "<<type2int(item1.type)<<endl;
        tmp->obj_t.push_back(type2int(item1.type));
        auto & item2=item1.pose;
        tmp->position.emplace_back(item2.position.x, item2.position.y);
        double x,y,z,w;
        x = item2.orientation.x;
        y = item2.orientation.y;
        z = item2.orientation.z;
        w = item2.orientation.w;
        if(n==1){
          tmp->theta.push_back(atan2(2*(z*w+y*x), 1-2*(z*z+y*y)));
        }
        else{
          tmp->theta.push_back(atan2(2*(z*w+y*x), 1-2*(z*z+y*y)));
        }
        if(z*z+w*w-y*y-x*x < 0){
          tmp->flipped.push_back(true);
        }
        else tmp->flipped.push_back(false);
      }
      tmp->finished.resize(tmp->position.size(),false);
      tmp->finished_2.resize(tmp->position.size(),false);
      tmp->invalid.resize(tmp->position.size(),false);
    }
  }

  bool reached(const double s[], const double t[]){
    double sum=0;
    for(int i=0;i<5/*6*/;i++){
      sum += min(fabs(s[i]-t[i]), fabs(6.28318530718 - fabs(s[i]-t[i])));
    }
    return sum < 3e-2;
  }
  bool reached_g(const double s[], const double t[]){
    double sum = 0;
    for(int i=0;i<3;i++){
      sum += fabs(s[i] - t[i]);
    }
    return sum < 3e-2;
  }

  vector<string> arm_1_joint_names{"left_shoulder_pan_joint", "left_shoulder_lift_joint", "left_elbow_joint", "left_wrist_1_joint", "left_wrist_2_joint", "left_wrist_3_joint"};
  vector<string> arm_2_joint_names{"right_shoulder_pan_joint", "right_shoulder_lift_joint", "right_elbow_joint", "right_wrist_1_joint", "right_wrist_2_joint", "right_wrist_3_joint"};
  void send_arm_to_state(const ros::Publisher & joint_trajectory_publisher, const double joints[], double t) {
    // Create a message to send.
    trajectory_msgs::JointTrajectory msg;

    if(joint_trajectory_publisher == arm_1_joint_trajectory_publisher_)
      memcpy(arm_1_joint_goal, joints, 6 * sizeof(double)), msg.joint_names = arm_1_joint_names;
    else
      memcpy(arm_2_joint_goal, joints, 6 * sizeof(double)), msg.joint_names = arm_2_joint_names;
    msg.points.resize(1);
    msg.points[0].positions.resize(msg.joint_names.size(), 0.0);
    for(int i=0;i<6;i++){
      msg.points[0].positions[i] = joints[i];
    }
    // How long to take getting to the point (floating point seconds).
    msg.points[0].time_from_start = ros::Duration(t);
    // ROS_INFO_STREAM("Sending command:\n" << msg);
    joint_trajectory_publisher.publish(msg);
  }
  void send_arm_to_states(const ros::Publisher & joint_trajectory_publisher, const double joints[][6], const double t[], const int n) {
    // Create a message to send.
    trajectory_msgs::JointTrajectory msg;

    if(joint_trajectory_publisher == arm_1_joint_trajectory_publisher_)
      memcpy(arm_1_joint_goal, joints[n-1], 6 * sizeof(double)), msg.joint_names = arm_1_joint_names;
    else
      memcpy(arm_2_joint_goal, joints[n-1], 6 * sizeof(double)), msg.joint_names = arm_2_joint_names;
    msg.points.resize(n);
    for(int i=0;i<n;i++){
      msg.points[i].positions.resize(msg.joint_names.size(), 0.0);
      for(int j=0;j<6;j++){
        msg.points[i].positions[j] = joints[i][j];
      }
      msg.points[i].time_from_start = ros::Duration(t[i]);
    }
    // How long to take getting to the point (floating point seconds).
    joint_trajectory_publisher.publish(msg);
  }
  void send_gantry_to_state(const ros::Publisher & joint_trajectory_publisher, const double joints[], const double t){
    trajectory_msgs::JointTrajectory msg;
    msg.joint_names.clear();
    msg.joint_names.push_back("small_long_joint");
    msg.joint_names.push_back("torso_base_main_joint");
    msg.joint_names.push_back("torso_rail_joint");
    memcpy(gantry_joint_goal, joints, 3 * sizeof(double));
    msg.points.resize(1);
    msg.points[0].positions.resize(msg.joint_names.size(), 0.0);
    for(int i=0;i<3;i++){
      msg.points[0].positions[i] = joints[i];
    }
    msg.points[0].time_from_start = ros::Duration(t);
    joint_trajectory_publisher.publish(msg);
  }
  void send_gantry_to_states(const ros::Publisher & joint_trajectory_publisher, const double joints[][3], const double t[], const int n){
    trajectory_msgs::JointTrajectory msg;
    msg.joint_names.clear();
    msg.joint_names.push_back("small_long_joint");
    msg.joint_names.push_back("torso_base_main_joint");
    msg.joint_names.push_back("torso_rail_joint");
    memcpy(gantry_joint_goal, joints[n-1], 3 * sizeof(double));
    msg.points.resize(n);
    for(int ii =0 ;ii<n;ii++){
      msg.points[ii].positions.resize(msg.joint_names.size(), 0.0);
      for(int i=0;i<3;i++){
        msg.points[ii].positions[i] = joints[ii][i];
      }
      msg.points[ii].time_from_start = ros::Duration(t[ii]);
    }
    joint_trajectory_publisher.publish(msg);
  }

  void logical_camera_1_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
    if(initialized[0]) return;
    for(const auto &item: image_msg->models){
      tf2::Quaternion tmp1(item.pose.orientation.x,item.pose.orientation.y,
        item.pose.orientation.z, item.pose.orientation.w);
      auto ori = q_logical*tmp1;
      ori.normalize();
      double x = ori.x(), y = ori.y(), z = ori.z(), w = ori.w();
      double px = lx[0] - item.pose.position.y;
      double py = ly[0] + item.pose.position.z;
      double pz = lz - item.pose.position.x;
      int type = type2int(item.type);
      int id = (px > lx[0] ? 1 : 0) + (py > ly[0] ? 4: 0);
      // if(bin_type[id] == 0) 
      int typ = type2int(item.type);
      bin_type[id] = typ;
      double theta = atan2(2 * (z*w+x*y), 1-2*(z*z-y*y));
      bool flipped = (1-2*(x*x + y*y)) < 0;
      part_pose[type].emplace_back(px, py, theta, flipped);
    }
    initialized[0] = true;
  }
  void logical_camera_2_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
    if(initialized[1]) return;
    for(const auto &item: image_msg->models){
      tf2::Quaternion tmp1(item.pose.orientation.x,item.pose.orientation.y,
        item.pose.orientation.z, item.pose.orientation.w);
      auto ori = q_logical*tmp1;
      ori.normalize();
      double x = ori.x(), y = ori.y(), z = ori.z(), w = ori.w();
      double px = lx[1] - item.pose.position.y;
      double py = ly[1] + item.pose.position.z;
      double pz = lz - item.pose.position.x;
      int type = type2int(item.type);
      int id = (px > lx[1] ? 1 : 0) + (py > ly[1] ? 6: 2);
      // if(bin_type[id] == 0) 
      int typ = type2int(item.type);
      double theta = atan2(2 * (z*w+x*y), 1-2*(z*z-y*y));
      bool flipped = (1-2*(x*x + y*y)) < 0;
      part_pose[type].emplace_back(px, py, theta, flipped);
      // cout<<px<<' '<<py<< ' '<<theta<<' '<<type<<endl;
    }
    initialized[1] = true;
  }
  void logical_camera_3_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
    if(initialized[2]) return;
    for(const auto &item: image_msg->models){
      tf2::Quaternion tmp1(item.pose.orientation.x,item.pose.orientation.y,
        item.pose.orientation.z, item.pose.orientation.w);
      auto ori = q_logical*tmp1;
      ori.normalize();
      double x = ori.x(), y = ori.y(), z = ori.z(), w = ori.w();
      double px = lx[2] - item.pose.position.y;
      double py = ly[2] + item.pose.position.z;
      double pz = lz - item.pose.position.x;
      int type = type2int(item.type);
      int id = (px > lx[2] ? 1 : 0) + (py > ly[2] ? 8: 12);
      // if(bin_type[id] == 0) 
      double theta = atan2(2 * (z*w+x*y), 1-2*(z*z-y*y));
      bool flipped = (1-2*(x*x + y*y)) < 0;
      part_pose[type].emplace_back(px, py, theta, flipped);
    }
    initialized[2] = true;
  }
  void logical_camera_4_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
    if(initialized[3]) return;
    for(const auto &item: image_msg->models){
      tf2::Quaternion tmp1(item.pose.orientation.x,item.pose.orientation.y,
        item.pose.orientation.z, item.pose.orientation.w);
      auto ori = q_logical*tmp1;
      ori.normalize();
      double x = ori.x(), y = ori.y(), z = ori.z(), w = ori.w();
      double px = lx[3] - item.pose.position.y;
      double py = ly[3] + item.pose.position.z;
      double pz = lz - item.pose.position.x;
      int type = type2int(item.type);
      int id = (px > lx[3] ? 1 : 0) + (py > ly[3] ? 10: 14);
      // if(bin_type[id] == 0) 
      // double theta = atan2(2 * (z*w+x*y), 1-2*(z*z-y*y));
      double theta = atan2(2 * (z*w+x*y), 1-2*(z*z-y*y));
      bool flipped = (1-2*(x*x + y*y)) < 0;
      part_pose[type].emplace_back(px, py, theta, flipped);
    }
    initialized[3] = true;
  }
  void logical_camera_5_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
    auto current_time = ros::Time::now();
    for(const auto &item: image_msg->models){
      if(item.pose.position.x < .92) continue;
      tf2::Quaternion tmp1(item.pose.orientation.x,item.pose.orientation.y,
        item.pose.orientation.z, item.pose.orientation.w);
      auto ori = q_logical*tmp1;
      ori.normalize();
      double x = ori.x(), y = ori.y(), z = ori.z(), w = ori.w();
      int type = type2int(item.type);
      double theta = atan2(2 * (z*w+x*y), 1-2*(z*z-y*y));
      bool flipped = (1-2*(x*x + y*y)) < 0;
      ros::Time to_z = current_time + ros::Duration((item.pose.position.z) / belt_vel);
      bool tag = false;
      for(int i=0;i<events.size();i++){
        if((to_z - events[i].st).toSec() < .1){
          tag=true;break;
        }
      }
      if(tag)continue;
      cout<<"coming height "<< item.pose.position.x<<endl;
      events.emplace_back(to_z, type, theta, flipped);
    }
  }

  // shelf
  void logical_camera_6_callback(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
    if(initialized[5]) return;
    for(const auto &item: image_msg->models){
      tf2::Quaternion tmp1(item.pose.orientation.x,item.pose.orientation.y,
        item.pose.orientation.z, item.pose.orientation.w);
      auto ori = q_logical*tmp1;
      ori.normalize();
      double x = ori.x(), y = ori.y(), z = ori.z(), w = ori.w();
      double px = lx[5] - item.pose.position.y;
      double py = ly[5] + item.pose.position.z;
      double pz = lz - item.pose.position.x;
      int type = type2int(item.type);
      double theta = atan2(2 * (z*w+x*y), 1-2*(z*z-y*y));
      bool flipped = (1-2*(x*x + y*y)) < 0;
      // part_pose[type].emplace_back(px, py, theta, flipped);
      part_pose_shelf[type].emplace_back(px, py, theta, flipped);
    }
    initialized[5] = true;
  }
  void agv(int num, string ship_t){
    nist_gear::AGVControl srv;
    srv.request.shipment_type = ship_t;
    // Sleep here
    if(num==1){
      agv_1.call(srv);
    }
    else{
      agv_2.call(srv);
    }
  }

  void gripper_1_callback(const nist_gear::VacuumGripperState::ConstPtr & msg){
    catched_1 = msg->attached;
    enabled_1 = msg->enabled;
  }

  void gripper_2_callback(const nist_gear::VacuumGripperState::ConstPtr & msg){
    catched_2 = msg->attached;
    enabled_2 = msg->enabled;
  }
  bool do_it(){
    // 1. event should be dealt with first
    vector<pair<Shipment*, bool>> tmp_list;
    if(shipments_1.size()) tmp_list.emplace_back(&shipments_1[0], false);
    if(shipments_2.size()) tmp_list.emplace_back(&shipments_2[0], true);
    for(int i=0;i<events.size();i++){
      for(auto [tmp, side]: tmp_list){
        // tmp = &shipments_1[0];
        for(int j=0;j<tmp->obj_t.size();j++){
          if(tmp->finished[j] || tmp->invalid[j] == true)continue;
          if(tmp->obj_t[j] == events[i].type){
            double t[4] = {3, 4.5, 5.5, 7};
            if(!catched_1){
              open_gripper(1);
              send_arm_to_states(arm_1_joint_trajectory_publisher_, q_sol_belt_1[(events[i].type-1)/3], t, 4);
              double nn = (ros::Time::now() - events[i].st).toSec();
              double gan[4][3] = {
                {- R - .1, 0, -ly[4] + (nn+3) * belt_vel}, 
                {- R - .1, 0, -ly[4] + (nn+4.5) * belt_vel}, 
                {- R - .1, 0, -ly[4] + (nn+5.5) * belt_vel}, 
                {- R - .1, 0, -ly[4] + (nn+7) * belt_vel}
              };
              send_gantry_to_states(gantry_joint_trajectory_publisher_, gan, t, 4);
              l_id = j;
              l_side = side;
              l_angle = events[i].theta;
              tmp->finished[j] = true;return true;
            }
            else if(!catched_2){
              open_gripper(2);
              send_arm_to_states(arm_2_joint_trajectory_publisher_, q_sol_belt_2[(events[i].type-1)/3], t, 4);
              double nn = (ros::Time::now() - events[i].st).toSec();
              double gan[4][3] = {
                {R + .1, 0, -ly[4] + (nn+3) * belt_vel}, 
                {R + .1, 0, -ly[4] + (nn+4.5) * belt_vel}, 
                {R + .1, 0, -ly[4] + (nn+5.5) * belt_vel}, 
                {R + .1, 0, -ly[4] + (nn+7) * belt_vel}
              };
              send_gantry_to_states(gantry_joint_trajectory_publisher_, gan, t, 4);
              r_id = j;
              r_angle = events[i].theta;
              r_side = side;
              tmp->finished[j] = true; return true;
            }
          }
        }
      }
    }
    // 2. bin 
    for(auto [tmp, side]: tmp_list){
      for(int i=0;i<tmp->obj_t.size();i++){
        if(tmp->finished[i] == true || tmp->invalid[i] == true) continue;
        int ptype = tmp->obj_t[i];
        if(part_pose[ptype].size()){
          double x = part_pose[ptype][0].x, y = part_pose[ptype][0].y;
          if(!catched_1){
            open_gripper(1);
            double gan[3] = {
              x - R - .1, 0, -y
            };
            double t_begin = max(fabs(gan[0] - gantry_joint[0]) / rail_vel, fabs(gan[2] - gantry_joint[2]) / base_vel);
            t_begin = max(t_begin, 1.7);
            double t[4] = {t_begin, t_begin + 1, t_begin + 2, t_begin + 3};
            send_arm_to_states(arm_1_joint_trajectory_publisher_, q_sol_bin_1[(ptype-1)/3], t, 4);
            send_gantry_to_state(gantry_joint_trajectory_publisher_, gan , t_begin);
            l_id = i, l_side = side;
            l_angle = part_pose[ptype][0].theta;
            tmp->finished[i] = true;
            part_pose_del[ptype].push_back(part_pose[ptype].front());
            part_pose[ptype].pop_front();
            return true;
          }else if(!catched_2){
            open_gripper(2);
            double gan[3] = {
              x + R + .1, 0, -y
            };
            double t_begin = max(fabs(gan[0] - gantry_joint[0]) / rail_vel, fabs(gan[2] - gantry_joint[2]) / base_vel);
            t_begin = max(t_begin, 1.7);
            double t[4] = {t_begin, t_begin + 1, t_begin + 2, t_begin + 3};
            send_arm_to_states(arm_2_joint_trajectory_publisher_, q_sol_bin_2[(ptype-1)/3], t, 4);
            send_gantry_to_state(gantry_joint_trajectory_publisher_, gan ,t_begin);
            r_id = i, r_side = side;
            r_angle = part_pose[ptype][0].theta;
            tmp->finished[i] = true;
            part_pose_del[ptype].push_back(part_pose[ptype].front());
            part_pose[ptype].pop_front();
            return true;
          }
        }
      }
    }
    // 3. shelf
    for(auto [tmp, side]: tmp_list){
      for(int i=0;i<tmp->obj_t.size();i++){
        if(tmp->finished[i] == true || tmp->invalid[i] == true) continue;
        int ptype = tmp->obj_t[i];
        if(part_pose_shelf[ptype].size()){
          if(!catched_1){
            open_gripper(1);
            double x = part_pose_shelf[ptype][0].x, y = part_pose_shelf[ptype][0].y;
            double gans[6][3] = {
              {x - R - .1, 0, - (y - (y>0 ? 2: -2))},
              {x - R - .1, 0, - (y - (y>0 ? 0.9: -0.9))},
              {x - R - .1, 0, - (y - (y>0 ? 0.9: -0.9))},
              {x - R - .1, 0, - (y - (y>0 ? 0.9: -0.9))},
              {x - R - .1, 0, - (y - (y>0 ? 0.9: -0.9))},
              {x - R - .1, 0, - (y - (y>0 ? 2: -2))},
            };
            double t_begin = max(fabs(gans[0][0] - gantry_joint[0]) / rail_vel, fabs(gans[0][2] - gantry_joint[2]) / base_vel);
            t_begin = max(t_begin, 2.0);
            double t[6] = {t_begin, t_begin + 1, t_begin + 2, t_begin + 3, t_begin + 4, t_begin + 4.5};
            send_arm_to_states(arm_1_joint_trajectory_publisher_, q_sol_shelf_1[y<0?1:0][(ptype-1)/3], t, 6);
            send_gantry_to_states(gantry_joint_trajectory_publisher_, gans , t, 6);
            l_id = i, l_side = side;
            l_angle = part_pose_shelf[ptype][0].theta;
            tmp->finished[i] = true;
            part_pose_del[ptype].push_back(part_pose[ptype].front());
            part_pose_shelf[ptype].pop_front();
            return true;
          }else if(!catched_2){
            open_gripper(2);
            double x = part_pose_shelf[ptype][0].x, y = part_pose_shelf[ptype][0].y;
            double gans[6][3] = {
              {x + R + .1, 0, - (y - (y>0 ? 2: -2))},
              {x + R + .1, 0, - (y - (y>0 ? 0.9: -0.9))},
              {x + R + .1, 0, - (y - (y>0 ? 0.9: -0.9))},
              {x + R + .1, 0, - (y - (y>0 ? 0.9: -0.9))},
              {x + R + .1, 0, - (y - (y>0 ? 0.9: -0.9))},
              {x + R + .1, 0, - (y - (y>0 ? 2: -2))}
            };
            double t_begin = max(fabs(gans[0][0] - gantry_joint[0]) / rail_vel, fabs(gans[0][2] - gantry_joint[2]) / base_vel);
            t_begin = max(t_begin, 2.0);
            double t[6] = {t_begin, t_begin + 1, t_begin + 2, t_begin + 3, t_begin + 4, t_begin + 4.5};
            send_arm_to_states(arm_2_joint_trajectory_publisher_, q_sol_shelf_2[y<0?1:0][(ptype-1)/3], t, 6);
            send_gantry_to_states(gantry_joint_trajectory_publisher_, gans , t, 6);
            r_id = i, r_side = side;
            r_angle = part_pose_shelf[ptype][0].theta;
            tmp->finished[i] = true;
            part_pose_del[ptype].push_back(part_pose[ptype].front());
            part_pose_shelf[ptype].pop_front();
            return true;
          }
        }
      }
    }
    // 4. invalid
    // if id = -1 and catched, it means it need to be put back
    // cannot co exist with normal catch?

    if(l_id !=-1 || r_id != -1) return false;
    for(auto [tmp, side]: tmp_list){
      for(int i=0;i<tmp->obj_t.size();i++){
        if(!tmp->finished_2[i] || !tmp->invalid[i])continue;
        auto [x, y] = (side? shipments_2[0]: shipments_1[0]).position[i];
        int ptype = tmp->obj_t[i];
        if(!catched_1){ // l_side = 0
          double T[12] = {
            1, 0, 0, 0.95,
            0, 1, 0, (side ? y - 0.214603 : -y + 0.214603),
            0, 0, 1, .1,
          };
          double gan[2][3] = {
            {-R + (side ? x: -x) - .1, 0, (side? 2.5: -2.5) },
            {-R + (side ? x: -x) - .1, 0, (side? 6.9: -6.9)},
          };
          double dtheta = (side ? shipments_2[0].theta[i]: -shipments_1[0].theta[i]);
          double q_sol[4][6];
          double t1[4] = {5, 6, 7, 8};
          double t2[2] = {2, 5};
          
          inverse(T, q_sol[0], 0);
          T[3] = 1.26 - part_height[(ptype-1)/3];
          inverse(T, q_sol[1], 0);
          inverse(T, q_sol[2], 0);
          T[3] = 0.9;
          inverse(T, q_sol[3], 0);
          open_gripper(1);
          for(int i=0;i<3;i++){
            q_sol[i][5] -= dtheta + (r_side? 0: PI);
            while(q_sol[i][5]>PI) q_sol[i][5] -= PI_2;
            while(q_sol[i][5]<-PI) q_sol[i][5] += PI_2;
          }
          if(fabs(gantry_joint[0]) > 2 || fabs(gantry_joint[2] - (side ? 6.9 : - 6.9)) > 3  ){
            cout<<gantry_joint[0] << " Longer1 " << gantry_joint[2]<<endl;
            send_arm_to_states(arm_1_joint_trajectory_publisher_, q_sol, t1, 4);
            send_gantry_to_states(gantry_joint_trajectory_publisher_, gan, t2, 2);
          }else{
            t1[0] -= 4,  t1[1] -= 4, t1[2] -= 4, t1[3] -= 4;
            send_arm_to_states(arm_1_joint_trajectory_publisher_, q_sol, t1, 4);
            send_gantry_to_state(gantry_joint_trajectory_publisher_, gan[1], 1);
          }
          l_invalid = true;
          left_p = ptype;
          tmp->finished[i] = false;
          tmp->finished_2[i] = false;
          return true;
        }
        else if(!catched_2){
          double T[12] = {
            -1, 0, 0, -0.95,
            0, 1, 0, (side ? y - 0.214603 : -y + 0.214603),
            0, 0, -1, .1,
          };
          double gan[2][3] = {
            {R + .1 - (side ? -x: x), 0, (side? 2.5: -2.5) },
            {R + .1 - (side ? -x: x), 0, (side? 6.9: -6.9)/*-7.114603*/},
          };
          double dtheta = (side ? shipments_2[0].theta[i]: -shipments_1[0].theta[i]);
          double q_sol[4][6];
          double t1[4] = {5, 6, 7, 8};
          double t2[2] = {2, 5};
          
          inverse(T, q_sol[0], 0);
          T[3] = -1.26 + part_height[(ptype-1)/3];
          inverse(T, q_sol[1], 0);
          inverse(T, q_sol[2], 0);
          T[3] = -0.9;
          inverse(T, q_sol[3], 0);
          open_gripper(2);
          for(int i=0;i<3;i++){
            q_sol[i][5] -= dtheta + (r_side? 0: PI);
            while(q_sol[i][5]>PI) q_sol[i][5] -= PI_2;
            while(q_sol[i][5]<-PI) q_sol[i][5] += PI_2;
          }
          if(fabs(gantry_joint[0]) > 2 || fabs(gantry_joint[2] - (side ? 6.9 : - 6.9)) > 3  ){
            cout<<gantry_joint[0] << " Longer2 " << gantry_joint[2]<<endl;
            send_arm_to_states(arm_2_joint_trajectory_publisher_, q_sol, t1, 4);
            send_gantry_to_states(gantry_joint_trajectory_publisher_, gan, t2, 2);
          }else{
            t1[0] -= 4,  t1[1] -= 4, t1[2] -= 4, t1[3] -= 4;
            send_arm_to_states(arm_2_joint_trajectory_publisher_, q_sol, t1, 4);
            send_gantry_to_state(gantry_joint_trajectory_publisher_, gan[1], 1);
          }
          r_invalid = true;
          right_p = ptype;
          tmp->finished[i] = false;
          tmp->finished_2[i] = false;
          return true;
        }
      }
    }
    return false;
  }
  bool place_it(){
    if(l_id!=-1){
      // cout<<"HERE"<<endl;
      if(cnt_1==0){
        // cout<<"HERE cnt = 0"<<endl;
        auto [x, y] = (l_side? shipments_2[0]: shipments_1[0]).position[l_id];
        double q_sol[2][6];
        // Time stamp to be modified !!!!!
        double t1[2] = {5, 7};
        double t2[2] = {2, 5};
        double T[12] = {
          1, 0, 0, 0.7,
          0, 1, 0, (l_side ? y - 0.214603 : -y + 0.214603),
          0, 0, 1, .1,
        };
        // cout<<"y = "<<T[7]<<endl;
        double gan[2][3] = {
          {0/*-R + (l_side ? x: -x) - .1*/, 0, (l_side? 2.5: -2.5) },
          {-R + (l_side ? x: -x) - .1, 0, (l_side? 6.9: -6.9)/*-7.114603*/},
        };
        inverse(T, q_sol[0], 0);
        T[3] = 1.05;
        inverse(T, q_sol[1], 0);
        double dtheta = (l_side ? shipments_2[0].theta[l_id]: -shipments_1[0].theta[l_id]) - l_angle;
        for(int i=0;i<3;i++){
          q_sol[i][5] -= dtheta + (l_side? 0: PI);
          while(q_sol[i][5]>PI) q_sol[i][5] -= PI_2;
          while(q_sol[i][5]<-PI) q_sol[i][5] += PI_2;
        }
        // TODO : calculate t
        if(fabs(gantry_joint[0]) < 2 || fabs(gantry_joint[2] - (l_side ? 6.9 : - 6.9)) > 3  ){
          send_arm_to_states(arm_1_joint_trajectory_publisher_, q_sol, t1, 2);
          send_gantry_to_states(gantry_joint_trajectory_publisher_, gan, t2, 2);
        }else{
          t1[0] -= 4;
          t1[1] -= 4;
          send_arm_to_states(arm_1_joint_trajectory_publisher_, q_sol, t1, 2);
          send_gantry_to_state(gantry_joint_trajectory_publisher_, gan[1], 1);
        }
        cnt_1++;
      }else if(cnt_1==1){
        close_gripper(1);
        auto [x, y] = (l_side? shipments_2[0]: shipments_1[0]).position[l_id];
        double T[12] = {
          1, 0, 0, 1.0,
          0, 1, 0, (l_side ? y - 0.214603 : -y + 0.214603),
          0, 0, 1, .1,
        };
        double q_sol[6];
        inverse(T, q_sol, 0);
        send_arm_to_state(arm_1_joint_trajectory_publisher_, q_sol, 1);
        cnt_1++;
      }else if(cnt_1==2){
        Shipment * tmp = l_side ? &shipments_2[0]: &shipments_1[0];
        // cout<<faul_1 << ' '<<l_side <<' '<<faul_2<<endl;
        if(catched_1){
          cnt_1 ++;
          return true;
        }
        if(faul_2 && l_side || faul_1 && !l_side){
          cout<<"Caught faulty part"<<endl;
          open_gripper(1);
          double q_sol[4][6];
          double x = faul_1_x, y = faul_1_y;
          if(faul_2 && l_side){
            x = faul_2_x;
            y = faul_2_y;
          }
          cout<<x<<' '<<y<<endl;
          double T[12] = {
            1, 0, 0, 1.1,
            0, 1, 0, l_side? y - 0.564603 : y + 0.564603,
            0, 0, 1, .1,
          };
          double gan[3] = {-R + x - .1, 0, (l_side? 6.9: -6.9)/*-7.114603*/};
          inverse(T, q_sol[0], 0);
          T[3] = 1.225 - part_height[(tmp->obj_t[l_id]-1)/3];
          inverse(T, q_sol[1], 0);
          inverse(T, q_sol[2], 0);
          T[3] = 1.05;
          inverse(T, q_sol[3], 0);
          // for(int i=0;i<4;i++){for(int j=0;j<6;j++) cout<<q_sol[i][j]<<' ';cout<<endl;}
          double t[4] = {1, 2 ,3 ,4};
          send_arm_to_states(arm_1_joint_trajectory_publisher_, q_sol, t, 4);
          send_gantry_to_state(gantry_joint_trajectory_publisher_, gan, 1);
          tmp->finished[l_id] = false;
          tmp->finished_2[l_id] = false;
          cnt_1 ++ ;
          return true;
        }
        // send_gantry_to_state(gantry_joint_trajectory_publisher_, q, 3);
        bool tag = true;
        
        tmp->finished_2[l_id] = true;
        for(int i=0;i<tmp->finished.size();i++){
          if(tmp->finished_2[i]==false || tmp->invalid[i]){tag=false;break;}
        }
        if(tag) agv(l_side ? 2: 1, l_side ? shipments_2[0].shipment_t: shipments_1[0].shipment_t);
        cnt_1 = 0;
        l_id = -1;
      }else if (cnt_1==3){
        if(!catched_1){
          cnt_1=2;
          return true;
        }else{
          cout<<"HERE"<<endl;
          double gan[3] = {R, 0, l_side? 6.9: -6.9};
          send_gantry_to_state(gantry_joint_trajectory_publisher_, gan , 0.5);
          cnt_1++;
        }
      }else if(cnt_1 < 16){
        close_gripper(1);
        cnt_1 ++;
      }else{
        cnt_1 = 0;
        l_id = -1;
      }
      return true;
    }else if(r_id!=-1){
      if(cnt_2==0){
        auto [x, y] = (r_side? shipments_2[0]: shipments_1[0]).position[r_id];
        double q_sol[2][6];
        double t1[2] = {5, 7};
        double t2[2] = {2, 5};
        double T[12] = {
          -1, 0, 0, -0.7,
          0, 1, 0, (r_side ? y - 0.214603: -y + 0.214603),
          0, 0, -1, .1//(r_side ? -x: x),
        };
        // Time stamp to be modified !!!!!
        double gan[2][3] = {
          {0/*R + .1 - (r_side ? -x: x)*/, 0, (r_side? 2.5: -2.5) },
          {R + .1 - (r_side ? -x: x), 0, (r_side? 6.9: -6.9)/*-7.114603*/},
        };
        inverse(T, q_sol[0], 0);
        T[3] = -1.05;
        inverse(T, q_sol[1], 0);
        double dtheta = (r_side ? shipments_2[0].theta[r_id]: -shipments_1[0].theta[r_id]) - r_angle;
        for(int i=0;i<3;i++){
          q_sol[i][5] -= dtheta + (r_side? 0: PI);
          while(q_sol[i][5]>PI) q_sol[i][5] -= PI_2;
          while(q_sol[i][5]<-PI) q_sol[i][5] += PI_2;
        }
        if(fabs(gantry_joint[0]) > 2 || fabs(gantry_joint[2] - (r_side ? 6.9 : - 6.9)) > 3  ){
          send_arm_to_states(arm_2_joint_trajectory_publisher_, q_sol, t1, 2);
          send_gantry_to_states(gantry_joint_trajectory_publisher_, gan, t2, 2);
        }else{
          t1[0] -= 4;
          t1[1] -= 4;
          send_arm_to_states(arm_2_joint_trajectory_publisher_, q_sol, t1, 2);
          send_gantry_to_state(gantry_joint_trajectory_publisher_, gan[1], 1);
        }
        cnt_2++;
      }else if(cnt_2==1){
        close_gripper(2);
        auto [x, y] = (r_side? shipments_2[0]: shipments_1[0]).position[r_id];
        double T[12] = {
          -1, 0, 0, -1.0,
          0, 1, 0, (r_side ? y - 0.214603: -y + 0.214603),
          0, 0, -1, .1//(r_side ? -x: x),
        };
        double q_sol[6];
        inverse(T, q_sol, 0);
        send_arm_to_state(arm_2_joint_trajectory_publisher_, q_sol, 1);
        cnt_2++;
      }else if(cnt_2==2){
        Shipment * tmp = r_side ? &shipments_2[0]: &shipments_1[0];
        if(catched_2){
          cnt_2 ++;
          return true;
        }
        if(faul_2 && r_side || faul_1 && !r_side){
          cout<<"Caught faulty part"<<endl;
          open_gripper(2);
          double q_sol[4][6];
          double x = faul_2_x, y = faul_2_y;
          if(faul_1 && !r_side){
            x = faul_1_x, y = faul_1_y;
          }
          double T[12] = {
            -1, 0, 0, -1.1,
            0, 1, 0, r_side ? y - 0.564603: y + 0.564603,
            0, 0, -1, .1,
          };
          double gan[3] = {R + x + .1, 0, (r_side? 6.9: -6.9)};
          inverse(T, q_sol[0], 0);
          T[3] = -1.225 + part_height[(tmp->obj_t[r_id]-1)/3];
          inverse(T, q_sol[1], 0);
          inverse(T, q_sol[2], 0);
          T[3] = -1.05;
          inverse(T, q_sol[3], 0);
          for(int i=0;i<4;i++){for(int j=0;j<6;j++)cout<< q_sol[i][j] <<' ';cout<<endl;}
          double t[4] = {1, 2, 3, 4};
          send_arm_to_states(arm_2_joint_trajectory_publisher_, q_sol, t, 4);
          send_gantry_to_state(gantry_joint_trajectory_publisher_, gan, 1);
          tmp->finished[r_id] = false;
          tmp->finished_2[r_id] = false;
          cnt_2 ++ ;
          return true;
        }
        tmp->finished_2[r_id] = true;
        bool tag= true;
        for(int i=0;i<tmp->finished.size();i++){
          if(tmp->finished_2[i]==false || tmp->invalid[i]){tag=false;break;}
        }
        if(tag) agv(r_side ? 2 : 1, r_side ? shipments_2[0].shipment_t: shipments_1[0].shipment_t);
        cnt_2 = 0;
        r_id = -1;
      }else if(cnt_2 == 3){
        if(!catched_2){
          cnt_2 = 2; return true;
        }else{
          double gan[3] = {-R, 0, r_side? 6.9: -6.9};
          send_gantry_to_state(gantry_joint_trajectory_publisher_, gan, .5);
          cnt_2 ++;
        }
      }else if(cnt_2 < 16){
        close_gripper(2);
        cnt_2 ++;
      }else {
        r_id = -1;
        cnt_2 = 0;
      }
      return true;
    }
    // put it back ?
    else if(l_invalid){
      cout<<"invalid here1"<<endl;
      if(cnt_1 == 0){
        double x = part_pose_del[left_p].back().x, y = part_pose_del[left_p].back().y;
        double gan[3] = {
          x - R - .1, 0, -y
        };
        double t_begin = max(max(fabs(gan[0] - gantry_joint[0]) / rail_vel, fabs(gan[2] - gantry_joint[2]) / base_vel), 1.7);
        double t[2] = {t_begin, t_begin + 1};
        if(fabs(y) > 3){
          double gans[2][3] = {
            {x - R - .1, 0, - (y - (y>0? 2: -2))},
            {x - R - .1, 0, - (y - (y>0? 0.9: -0.9))}
          };
          double tt[3] = {t_begin, t_begin + 1, t_begin + 2};
          send_arm_to_states(arm_1_joint_trajectory_publisher_, q_sol_shelf_1_put[y<0? 1: 0], tt, 3);
          send_gantry_to_states(gantry_joint_trajectory_publisher_, gans ,tt, 2);
        }
        else {
          send_arm_to_states(arm_1_joint_trajectory_publisher_, q_sol_bin_1_put, t, 2);
          send_gantry_to_state(gantry_joint_trajectory_publisher_, gan ,t_begin);
        }
        cnt_1 ++;
        return true;
      }else if(cnt_1 == 1){
        close_gripper(1);
        double y = part_pose_del[left_p].back().y;
        if(fabs(y) < 3) part_pose[left_p].push_back(part_pose_del[left_p].back());
        else part_pose_shelf[left_p].push_back(part_pose_del[left_p].back());
        part_pose_del[left_p].pop_back();
        l_invalid = false;
        cnt_1 ++;
        if(fabs(y) <3) send_arm_to_state(arm_1_joint_trajectory_publisher_, q_sol_bin_1_put[0], 1);
        else send_arm_to_state(arm_1_joint_trajectory_publisher_, q_sol_shelf_1_put[y<0?1:0][0], 1);
        return true;
      }else{
        if(fabs(arm_1_joint[0]) > PI/2){
          double gan[3] = {gantry_joint[0], gantry_joint[1], gantry_joint[2] - (gantry_joint[2] > 0 ? 1: -1)};
          send_gantry_to_state(gantry_joint_trajectory_publisher_, gan, 1);
          send_arm_to_state(arm_1_joint_trajectory_publisher_, rest_joints_1, 1);
        }
        cnt_1=0;return true;
      }
    }else if(r_invalid){
      cout<<"invalid here2"<<endl;
      if(cnt_2 == 0){
        double x = part_pose_del[right_p].back().x, y = part_pose_del[right_p].back().y;
        double gan[3] = {
          x + R + .1, 0, -y
        };
        double t_begin = max(max(fabs(gan[0] - gantry_joint[0]) / rail_vel, fabs(gan[2] - gantry_joint[2]) / base_vel), 1.7);
        double t[2] = {t_begin, t_begin + 1};
        if(fabs(y) > 3){
          double gans[2][3] = {
            {x + R + .1, 0, - (y - (y>0? 2: -2))},
            {x + R + .1, 0, - (y - (y>0? 0.9: -0.9))}
          };
          double tt[3] = {t_begin, t_begin + 1, t_begin + 2};
          send_arm_to_states(arm_2_joint_trajectory_publisher_, q_sol_shelf_2_put[y<0? 1: 0], tt, 3);
          send_gantry_to_states(gantry_joint_trajectory_publisher_, gans , tt, 2);
        }
        else {
          send_arm_to_states(arm_2_joint_trajectory_publisher_, q_sol_bin_2_put, t, 2);
          send_gantry_to_state(gantry_joint_trajectory_publisher_, gan ,t_begin);
        }
        cnt_2 ++;
        return true;
      }else if(cnt_2 == 1){
        close_gripper(2);
        double y = part_pose_del[right_p].back().y;
        if(fabs(y)<3) part_pose[right_p].push_back(part_pose_del[right_p].back());
        else part_pose_shelf[right_p].push_back(part_pose_del[right_p].back());
        part_pose_del[right_p].pop_back();
        r_invalid = false;
        cnt_2 ++;
        if(fabs(y)<3) send_arm_to_state(arm_2_joint_trajectory_publisher_, q_sol_bin_2_put[0], 1);
        else send_arm_to_state(arm_2_joint_trajectory_publisher_, q_sol_shelf_2_put[y<0?1:0][0], 1);
        return true;
      }else{
        if(fabs(arm_2_joint[0]) > PI/2){
          double gan[3] = {gantry_joint[0], gantry_joint[1], gantry_joint[2] - (gantry_joint[2] > 0 ? 1: -1)};
          send_gantry_to_state(gantry_joint_trajectory_publisher_, gan, 1);
          send_arm_to_state(arm_2_joint_trajectory_publisher_, rest_joints_2, 1);
        }
        cnt_2 = 0; return true;   
      }
    }
    return false;
  }
  bool flip_it(){
  }
  void gantry_joint_states_callback(const sensor_msgs::JointState::ConstPtr & joint_state_msg){
    // ROS_INFO_STREAM_THROTTLE(10,
    //    "Joint States arm 1 (throttled to 0.1 Hz):\n" << *joint_state_msg);
    // ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
    const vector<double>& gantry_current_joint_states_ = joint_state_msg->position;
    arm_1_joint[0] = gantry_current_joint_states_[2]; //shoulder pan
    arm_1_joint[1] = gantry_current_joint_states_[1]; //shoulder lift
    arm_1_joint[2] = gantry_current_joint_states_[0]; //elbow
    arm_1_joint[3] = gantry_current_joint_states_[4]; //wrist 1
    arm_1_joint[4] = gantry_current_joint_states_[5]; //wrist 2
    arm_1_joint[5] = gantry_current_joint_states_[6]; //wrist 3

    arm_2_joint[0] = gantry_current_joint_states_[9]; //shoulder pan
    arm_2_joint[1] = gantry_current_joint_states_[8]; //shoulder lift
    arm_2_joint[2] = gantry_current_joint_states_[7]; //elbow
    arm_2_joint[3] = gantry_current_joint_states_[11]; //wrist 1
    arm_2_joint[4] = gantry_current_joint_states_[12]; //wrist 2
    arm_2_joint[5] = gantry_current_joint_states_[13]; //wrist 3
    
    gantry_joint[0] = gantry_current_joint_states_[14]; //small_long_joint
    gantry_joint[1] = gantry_current_joint_states_[15]; //torso_base_main_joint
    gantry_joint[2] = gantry_current_joint_states_[16]; //torso_rail_joint
    
    if(!arm_1_has_been_zeroed_){
      send_arm_to_state(arm_1_joint_trajectory_publisher_, rest_joints_1, 0.01);
      if(reached(arm_1_joint_goal, arm_1_joint)){
        arm_1_has_been_zeroed_ = true;
      }
    }
    if(!arm_2_has_been_zeroed_){
      send_arm_to_state(arm_2_joint_trajectory_publisher_, rest_joints_2, 0.01);
      if(reached(arm_2_joint_goal, arm_2_joint)){
        arm_2_has_been_zeroed_ = true;
      }
    }
    if(!gantry_has_been_zeroed_){
      double j[3] ={0, 0, 0};
      send_gantry_to_state(gantry_joint_trajectory_publisher_, j, 0.01);
      if(reached_g(gantry_joint, gantry_joint_goal)) gantry_has_been_zeroed_ = true;
    }
    if(!arm_1_has_been_zeroed_ || !arm_2_has_been_zeroed_ || !gantry_has_been_zeroed_) return;

    switch(ST){
      case IDLE:
      // to be modified according to each order's starting time
      if((ros::Time::now() - start_stamp).toSec() > 300){
        if(shipments_1.size() && !shipments_1[0].invalid[0]) agv(1, shipments_1[0].shipment_t);
        if(shipments_2.size() && !shipments_2[0].invalid[0]) agv(2, shipments_2[0].shipment_t);
      }
      if(!reached(arm_1_joint_goal, arm_1_joint) || !reached(arm_2_joint_goal, arm_2_joint) || !reached_g(gantry_joint, gantry_joint_goal)) return;
      cout<<"IDLE"<<endl;
      do_it();
      ST = ONE;
      
      break;
      case ONE:
      // TODO: handle drop
      if(!reached(arm_1_joint_goal, arm_1_joint) || !reached(arm_2_joint_goal, arm_2_joint) || !reached_g(gantry_joint, gantry_joint_goal)) return;
      if(l_id != -1 && !catched_1){
        // ST = IDLE; 
        (l_side? shipments_2[0] : shipments_1[0]).finished[l_id] = false;
        l_id = -1; 
        do_it();
        break;
      }else if(r_id != -1 && !catched_2){
        // ST = IDLE; 
        (r_side? shipments_2[0] : shipments_1[0]).finished[r_id] = false;
        r_id = -1; 
        do_it();
        break;
      }else if(l_invalid && !catched_1){
        do_it();
        break;
      }else if(r_invalid && !catched_2){
        do_it();
        break;
      }
      cout<<"ONE"<<endl;
      do_it();
      ST = TWO;
      break;
      // case INTER:{
      //   if(!reached(arm_1_joint_goal, arm_1_joint) || !reached(arm_2_joint_goal, arm_2_joint) || !reached_g(gantry_joint, gantry_joint_goal)) return;
      //   double q[3] = {0,0,0};
      //   send_gantry_to_state(gantry_joint_trajectory_publisher_, q, 3),ST = IDLE;
      //   break;
      // }
      case TWO: 
      // TODO: handle drop
      if(!reached(arm_1_joint_goal, arm_1_joint) || !reached(arm_2_joint_goal, arm_2_joint) || !reached_g(gantry_joint, gantry_joint_goal)) return;
      if(l_id != -1 && !catched_1 && cnt_1 == 0){
        (l_side? shipments_2[0] : shipments_1[0]).finished[l_id] = false;
        l_id = -1; 
        do_it();
        break;
      }else if(r_id != -1 && !catched_2 && cnt_2 == 0){
        (r_side? shipments_2[0] : shipments_1[0]).finished[r_id] = false;
        r_id = -1; 
        do_it();
        break;
      }else if(l_invalid && !catched_1 && cnt_1 == 0){
        do_it();
        break;
      }else if(r_invalid && !catched_2 && cnt_2 == 0){
        do_it();
        break;
      }
      if(shipments_1.size() && shipments_1[0].invalid[0]){
        bool tag = true;
        // what if update when one arm already has one object
        for(int i=0;i<shipments_1[0].finished_2.size();i++){
          if(shipments_1[0].finished_2[i]) tag=false;
        }
        if(tag) shipments_1.erase(shipments_1.begin());
      }
      if(shipments_2.size() && shipments_2[0].invalid[0]){
        bool tag = true;
        for(int i=0;i<shipments_2[0].finished_2.size();i++){
          if(shipments_2[0].finished_2[i]) tag=false;
        }
        if(tag) shipments_2.erase(shipments_2.begin());
      }
      cout<<"TWO"<<endl;
      if((l_invalid || r_invalid) && fabs(gantry_joint_goal[2]) > 3){
        double q[3] = {0,0,gantry_joint[2] < 0 ? -2 : 2};
        send_gantry_to_state(gantry_joint_trajectory_publisher_, q, 3);
        send_arm_to_state(arm_1_joint_trajectory_publisher_, rest_joints_1, 1);
        send_arm_to_state(arm_2_joint_trajectory_publisher_, rest_joints_2, 1);
        break;
      }
      if(!place_it()) {
        double q[3] = {0,0,gantry_joint[2] < 0 ? -2 : 2};
        send_gantry_to_state(gantry_joint_trajectory_publisher_, q, 2),ST = IDLE;
        send_arm_to_state(arm_1_joint_trajectory_publisher_, rest_joints_1, 1);
        send_arm_to_state(arm_2_joint_trajectory_publisher_, rest_joints_2, 1);
      }
      // case :

      break;
    }
  }

  void agv_1_callback(const std_msgs::String::ConstPtr &msg){
    
  }

  void agv_2_callback(const std_msgs::String::ConstPtr &msg){
    
  }

  void open_gripper(int num){
    nist_gear::VacuumGripperControl srv;
    srv.request.enable = true;
    if(num==1){
      arm_1_gripper_ctrl.call(srv);
    }
    else{
      arm_2_gripper_ctrl.call(srv);
    }
  }
  void close_gripper(int num){
    nist_gear::VacuumGripperControl srv;
    srv.request.enable = false;
    if(num==1){
      arm_1_gripper_ctrl.call(srv);
    }
    else{
      arm_2_gripper_ctrl.call(srv);
    }

    if(!srv.response.success){
      ROS_ERROR_STREAM("Gripper Failed");
    }
  }
  
  // quality sensor 1 is above agv 2 ...

  void quality_ctrl_2(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
    if(image_msg->models.size() == 0) faul_1 = false;
    else {
      faul_1 = true;
      faul_1_x = -image_msg->models[0].pose.position.y;
      faul_1_y = image_msg->models[0].pose.position.z;
    }
  }
  void quality_ctrl_1(const nist_gear::LogicalCameraImage::ConstPtr & image_msg){
    if(image_msg->models.size() == 0) faul_2 = false;
    else {
      faul_2 = true;
      faul_2_x = image_msg->models[0].pose.position.y;
      faul_2_y = -image_msg->models[0].pose.position.z;
    }
  }
private:
  ros::Publisher arm_1_joint_trajectory_publisher_;
  ros::Publisher arm_2_joint_trajectory_publisher_;
  ros::Publisher gantry_joint_trajectory_publisher_;
  
  ros::ServiceClient arm_1_gripper_ctrl;
  ros::ServiceClient arm_2_gripper_ctrl;
  ros::ServiceClient agv_1;
  ros::ServiceClient agv_2;

  const double belt_vel = 0.2;
  const double base_vel = 4;
  const double rail_vel = 4;

  double arm_1_joint[6], arm_2_joint[6], gantry_joint[3];

  double arm_1_joint_goal[6], arm_2_joint_goal[6], gantry_joint_goal[3];

  const tf2::Quaternion q_logical=tf2::Quaternion(-0.5, 0.5, 0.5, 0.5);

  const double bx[4] = {2.634189, 3.574991, 4.515793, 5.456594};
  const double by[4] = {-2.165594, -1.323536, 1.323536, 2.165594};
  const double lx[6] = {3.10459, 4.9861935, 3.10459, 4.9861935, 0, 3.2};
  const double ly[6] = {1.744565, 1.744565, -1.744565, -1.744565, 3.5, 3.64};
  const double lz = 1.82;

  int bin_type[16] = {0};
  bool initialized[6] = {false};
  deque<part_bin_pose> part_pose[13];
  deque<part_bin_pose> part_pose_del[13];
  deque<part_bin_pose> part_pose_shelf[13];
  bool manipulate[16] = {false};

  deque<part_belt_pose> events; 

  const double rest_joints_1[6] = {-0.192301, -0.733444, 1.56079, -0.827348, 1.3785, 3.14159};
  const double rest_joints_2[6] = {0.192301, -2.40815, -1.56079, -2.31424, -1.3785, 0};
  const double R = 0.5148;
  State ST = IDLE;
  int l_id=-1, r_id = -1;

  //side: left false, right true
  bool l_side = false, r_side = false;
  double l_angle=0, r_angle=0;

  int left_p=0, right_p = 0;

  int cnt_1 = 0, cnt_2 = 0;

  bool catched_1, catched_2;
  bool enabled_1, enabled_2;

  double q_sol_belt_1[4][4][6];
  double q_sol_belt_2[4][4][6];
  double q_sol_bin_1[4][4][6];
  double q_sol_bin_2[4][4][6];

  double q_sol_bin_1_put[2][6];
  double q_sol_bin_2_put[2][6];

  double q_sol_shelf_1[2][4][6][6];
  double q_sol_shelf_2[2][4][6][6];

  double q_sol_shelf_1_put[2][3][6];
  double q_sol_shelf_2_put[2][3][6];
  // double q_sol_tray_1[4][4][6];
  // double q_sol_tray_2[4][4][6];
  bool l_invalid=false, r_invalid=false;

  const double part_height[4] = {0.017, 0.0156, 0.006, 0.07}; // gasket gear piston rod pulley

  bool faul_1=false;
  int faul_1_t=0;
  double faul_1_x=0;
  double faul_1_y=0;

  bool faul_2=false;
  int faul_2_t=0;
  double faul_2_x=0;
  double faul_2_y=0;
};





// %Tag(MAIN)%
int main(int argc, char ** argv) {
  
  ros::init(argc, argv, "ariac_example_node");
  
  ros::NodeHandle node;

  MyCompetitionClass comp_class(node);

  //orders
  ros::Subscriber orders_subscriber = node.subscribe(
    "/ariac/orders", 10,
    &MyCompetitionClass::order_callback, &comp_class);

  //joint_states
  ros::Subscriber gantry_joint_states_controller_subscriber = node.subscribe(
    "/ariac/gantry/joint_states", 10,
    &MyCompetitionClass::gantry_joint_states_callback, &comp_class);

  //agv
  ros::Subscriber agv_1_state_subscriber = node.subscribe(
    "/ariac/agv1/state", 10,
    &MyCompetitionClass::agv_1_callback, &comp_class);

  ros::Subscriber agv_2_state_subscriber = node.subscribe(
    "/ariac/agv2/state", 10,
    &MyCompetitionClass::agv_2_callback, &comp_class);

  // sensors
  
  ros::Subscriber logical_camera_1_subscriber = node.subscribe(
    "/ariac/logical_camera_1", 10,
    &MyCompetitionClass::logical_camera_1_callback, &comp_class);
  ros::Subscriber logical_camera_2_subscriber = node.subscribe(
    "/ariac/logical_camera_2", 10,
    &MyCompetitionClass::logical_camera_2_callback, &comp_class);
  ros::Subscriber logical_camera_3_subscriber = node.subscribe(
    "/ariac/logical_camera_3", 10,
    &MyCompetitionClass::logical_camera_3_callback, &comp_class);
  ros::Subscriber logical_camera_4_subscriber = node.subscribe(
    "/ariac/logical_camera_4", 10,
    &MyCompetitionClass::logical_camera_4_callback, &comp_class);
  ros::Subscriber logical_camera_5_subscriber = node.subscribe(
    "/ariac/logical_camera_5", 10,
    &MyCompetitionClass::logical_camera_5_callback, &comp_class);
  ros::Subscriber logical_camera_6_subscriber = node.subscribe(
    "/ariac/logical_camera_6", 10,
    &MyCompetitionClass::logical_camera_6_callback, &comp_class);
  // ros::Subscriber laser_profiler_subscriber = node.subscribe(
  //   "/ariac/laser_profiler_1", 10, 
  //   &MyCompetitionClass::laser_profiler_callback, &comp_class);
  

  //gripper


  ros::Subscriber gripper_1_state_subscriber = node.subscribe(
    "/ariac/gantry/left_arm/gripper/state", 10, 
    &MyCompetitionClass::gripper_1_callback, &comp_class);

  ros::Subscriber gripper_2_state_subscriber = node.subscribe(
    "/ariac/gantry/right_arm/gripper/state", 10, 
    &MyCompetitionClass::gripper_2_callback, &comp_class);



  //faulty

  ros::Subscriber quality_ctrl_1_subscriber = node.subscribe(
    "/ariac/quality_control_sensor_1", 10,
    &MyCompetitionClass::quality_ctrl_1, &comp_class);  

  ros::Subscriber quality_ctrl_2_subscriber = node.subscribe(
    "/ariac/quality_control_sensor_2", 10,
    &MyCompetitionClass::quality_ctrl_2, &comp_class);
  
  ROS_INFO("Setup complete.");

  start_competition(node);
  
  ros::spin();  

  return 0;
}

