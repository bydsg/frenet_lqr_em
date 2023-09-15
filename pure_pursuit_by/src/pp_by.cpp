#include "ros/ros.h"
#include "la/path_la.h"
#include "la/path_la_point.h"
#include <nav_msgs/Odometry.h>
#include "tf/tf.h"
using namespace std;
void range_precote(auto &val, double max, double min)
{
  if (val > max)
  {
    val = max;
  }
  if (val < min)
  {
    val = min;
  }
}
struct Point {
    double x;
    double y;
    double l;
    double r;
    double s;
    double theta;
};
struct Road
{
    int id;
    std::vector<int> pre;
    std::vector<int> beh;
    double distance;
    std::vector<Point> road_points;

};
struct State{
        double x = 0;          // m
        double y = 0;          // m
        double yaw = 0;        // degree
        double speed = 0;      // m/s
        double yawrate = 0;
    };

class pp_by{
 public:
     ros::Publisher pub_control_cmd;
     la::path_la path_la;
     State vehicleState;
     bool first_flag=0;

     void node_start(int argc, char *argv[]);
     void track_follow_process(const State &s, la::path_la path_la);
     void la_path_callback(const la::path_la msg);
     void odometryGetCallBack(const nav_msgs::Odometry::ConstPtr odometry_msg);
};
void pp_by::track_follow_process(const State &s,la::path_la path_la) {
        double wheelAngle = 0;
        geometry_msgs::Twist cmd_vel;

          int index =1;//latisi::get_goal_index(s, path); // 在车当前最近点加上前视距离的道路的坐标点
        // 用上一个循环的目标点判断是否是在向前走
        la::path_la_point goal; 
        range_precote(index, path_la.points.size() - 1, 0);   //防止index溢出
        float dis_min=100000;
        for(int i=0;i<path_la.points.size();i++){
          float dis=pow(vehicleState.x-path_la.points[i].x,2)+pow(vehicleState.y-path_la.points[i].y,2);
          if(dis<dis_min){
            dis_min=dis;
            index=i;
          }
        }
        range_precote(index, path_la.points.size() - 1, 0);   //防止index溢出
        goal = path_la.points[index+1];
        double alpha = atan2( goal.y - s.y,goal.x - s.x)-s.yaw; // 误差角度
    
        if(alpha<-3.14) {alpha=alpha+6.28;}
        if(alpha>3.14) {alpha=alpha-6.28;};
        //判断路径是否走完
        wheelAngle =alpha;    
        //发布前轮转角
        cmd_vel.angular.z = wheelAngle;
        cmd_vel.linear.x = 1;
        cout << "cmd_vel.angular.z" << cmd_vel.angular.z << endl;
        this->pub_control_cmd.publish(cmd_vel);
    }
void pp_by::odometryGetCallBack(const nav_msgs::Odometry::ConstPtr odometry_msg){
  double raw, pitch, theta;
  tf::Quaternion q;
  tf::quaternionMsgToTF(odometry_msg->pose.pose.orientation, q);
  tf::Matrix3x3(q).getRPY(raw, pitch, theta);
  double g_velocity = std::sqrt(std::pow(odometry_msg->twist.twist.linear.x, 2) + std::pow(odometry_msg->twist.twist.linear.y, 2));
  this->vehicleState.x = odometry_msg->pose.pose.position.x;
  this->vehicleState.y = odometry_msg->pose.pose.position.y;
  this->vehicleState.speed = g_velocity;
  this->vehicleState.yaw = theta;
  this->vehicleState.yawrate = odometry_msg->twist.twist.angular.z;
    if(first_flag){
        track_follow_process(this->vehicleState, this->path_la);
    }

}
 void pp_by::la_path_callback(const la::path_la msg){
    path_la.points.clear();
    for(int i=0 ;i<msg.points.size();i++){
        la::path_la_point p;

        p.x = msg.points[i].x;
        p.y = msg.points[i].y;
        p.s = msg.points[i].s;
        p.s_dot = msg.points[i].s_dot;
        p.s_dot_dot = msg.points[i].s_dot_dot;
        p.s_dot_dot_dot = msg.points[i].s_dot_dot_dot;
        p.d = msg.points[i].d;
        p.d_dot = msg.points[i].d_dot;
        p.d_dot_dot = msg.points[i].d_dot_dot;
        p.d_dot_dot = msg.points[i].d_dot_dot_dot;
        p.heading = msg.points[i].heading;
        path_la.points.push_back(p);
    }
    first_flag=1;
 }
void pp_by::node_start(int argc, char  *argv[]){
    ros::init(argc, argv, "pure_pursuit");
    ros::NodeHandle n;

    pub_control_cmd = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);//发布cmd_vel

    ros::Subscriber sub_globle_path = n.subscribe("la2control", 1, &pp_by::la_path_callback, this); // 接受local规划路径
    ros::Subscriber sub_odom= n.subscribe("/odom", 1, &pp_by::odometryGetCallBack, this);        // 控制节点

    ros::spin();
}
int main(int argc, char  *argv[])
{
    pp_by pp;
    pp.node_start(argc,argv);
    return 0;
}
