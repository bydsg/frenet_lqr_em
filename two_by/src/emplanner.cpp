#include "ros/ros.h"
#include "by_djstl/PathPoint.h"
#include "by_djstl/Path.h"
#include <nav_msgs/Odometry.h>
#include "tf/tf.h"
#include <eigen3/Eigen/Dense>//引用Ｅｉｇｅｎ库
#include "iostream"
#include <visualization_msgs/MarkerArray.h>
#include "rviz.h"
#include "Box2d.h"
#include <qpOASES.hpp>
#include "control.h"
#include "std_msgs/String.h"
#include <ros/callback_queue.h>

#include "iostream"
using namespace qpOASES;
using namespace std;
using namespace Eigen;
//参考线平滑参数
#define x_ub 1
#define y_ub 1
#define x_lb 1
#define y_lb 1
#define w_cost_smooth 1000
#define w_cost_length 1
#define w_cost_ref 1
///////////////////////////////////
#define sample_s 1
#define sample_l 1
#define w_cost_collision 1
#define w_cost_smooth_dl 20
#define w_cost_smooth_ddl 10
#define w_cost_smooth_dddl 0
#define w_cost_ref_2 1
#define w_cost_ref_start 150
#define row 5
#define col 7



struct Point {
    double x;
    double y;
    double l;
    double r;
    double s;
    double theta;
};
struct Path_msg {
    vector<float> x;
    vector<float> y;
    vector<float> heading;
    vector<float> kappa;
    vector<float> s;
};
struct Point_msg {
    float x;
    float y;
    float heading;
    float kappa;
    float s;
    float l;
    float s_d;
    float l_d;
    float s_d_d;
    float l_d_d;
    float dl;
    float ddl;
    float v;
    float vx;
    float vy;
    float a;
    float ax;
    float ay;
    double time;
};
struct Point_vec_msg {
    vector<Point_msg> Point_msg_vec;
};
struct State{
        double x = 0;          // m
        double y = 0;          // m
        double yaw = 0;        // degree
        double speed = 0;      // m/s
        double yawrate = 0;
        double vx = 0;
        double vy = 0;
        double vx_last = 0;
        double vy_last = 0;
        double ax = 0;
        double ay = 0;
};

vector<float> wx_by,wy_by;

class em{
public:
    MatrixXd A1,A2,A3,f,lb,ub,H;
    ros::Timer timer ;

    ros::Publisher marker_pub;
    ros::Publisher marker_pub_start_point;
    ros::Publisher marker_pub_all_obs;
    ros::Publisher marker_pub_dp_point;
    ros::Publisher marker_pub_dp_point_choose;
    ros::Publisher pub_control_cmd;
    ros::Publisher pub_globle_path;

    bool get_gloab_point_flag=0;
    State vehicleState;
    Path_msg paht_msg;//参考线
    Path_msg paht_msg_proj;//投影点
    Path_msg paht_msg_referenceline_init;//投影点前后集合
    Path_msg paht_msg_final;//二次规划优化后的参考线
    Path_msg referenceline;
    int index_nearcar=0;

  
    void bc(const ros::TimerEvent& event);

    void osqp_referenceline_by();
    void nodeStart(int argc, char **argv);
    void globlepathGetCallBack( by_djstl::Path msg);
    void control_odometryGetCallBack(const nav_msgs::Odometry::ConstPtr odometry_msg);
    void get_odom_msge(const nav_msgs::Odometry::ConstPtr odometry_msg)
    {
        double raw, pitch, theta;
        tf::Quaternion q;
        tf::quaternionMsgToTF(odometry_msg->pose.pose.orientation, q);
        tf::Matrix3x3(q).getRPY(raw, pitch, theta);
        double g_velocity = std::sqrt(std::pow(odometry_msg->twist.twist.linear.x, 2) + std::pow(odometry_msg->twist.twist.linear.y, 2));
        vehicleState.x = odometry_msg->pose.pose.position.x;
        vehicleState.y = odometry_msg->pose.pose.position.y;
        vehicleState.speed = g_velocity;
        vehicleState.yaw = theta;
        vehicleState.yawrate = odometry_msg->twist.twist.angular.z;
        vehicleState.vx = odometry_msg->twist.twist.linear.x;
        vehicleState.vy = odometry_msg->twist.twist.linear.y;
        vehicleState.ax = odometry_msg->twist.twist.linear.x - vehicleState.vx_last;
        vehicleState.ay = odometry_msg->twist.twist.linear.x - vehicleState.vx_last;
        vehicleState.vx_last = odometry_msg->twist.twist.linear.x;
        vehicleState.vy_last = odometry_msg->twist.twist.linear.y;
    }
 
    void get_headingandkappa(vector<float> wx_by,vector<float> wy_by,Path_msg& save_msg);
    int  get_indexandmsg(Path_msg paht_msg,State vehicleState );//找到匹配点
    void get_touying_point();//找到投影点
    void find_road_scope(int pre,int beh);//找到起始点并获取数据
    /////////////////////////////////////////////////////////////////////////////////////////
    Point_msg plan_start_msg;
    Point_vec_msg paln_msg;
    Point_vec_msg car_msg;
    Point_vec_msg obs_msg;
    ros::Time current_time;
    MatrixXd index2s;
    Point_vec_msg dp_pathxy;
    float cost_node[row][col];
    float pre_node_index[row][col];
    bool find_start_point_first_flag=0;
    void rviz_start_point(ros::Publisher marker_pub, Point_msg plan_start_msg);

    void find_plan_start_point(double time_now);
    void cascsl(float set_x,float set_y,float set_vx,float set_vy,float set_ax,float set_ay,Point_vec_msg& point_vec_msg,int max_size);
    void obs_deal(vector<Box_2d> obstacles);
    void all_obs_rviz(ros::Publisher marker_pub, vector<Box_2d> vel_obs_info);
    void CalaStartCost( Point_vec_msg car_msg,Point_vec_msg obs_msg);
    Eigen::VectorXf CalcQuinticCoeffient(float start_l, float start_dl ,float start_ddl, float end_l, float end_dl, 
                               float end_ddl,float start_s, float end_s);
    float CalcNeighbourCost(float pre_node_s,float pre_node_l,float cur_node_s,float cur_node_l);
    Point_msg Calaxy(float s,float l);
    void make_dp_path();
    void draw_dp_point_and_path();
};


void em::control_odometryGetCallBack(const nav_msgs::Odometry::ConstPtr odometry_msg){
    if(get_gloab_point_flag){
        // ros::Rate rate(0.1);
        // get_odom_msge(odometry_msg); // 获取车的里程计数据
        // timer.start();
        // index_nearcar = get_indexandmsg(paht_msg, vehicleState); // 找到匹配点
        // get_touying_point();                                     // 找到投影点
        // find_road_scope(wx_by.size()-21, 20);                                // 取投影点前后部分
        // osqp_referenceline_by();                                 // 参考线二次规划,
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // for (int i = 0; i < row; i++){
        //     for (int j = 0; j < col; j++){
        //         cost_node[i][j] = 10000000000000;
        //     }
        // }
        // current_time = ros::Time::now();
        // find_plan_start_point(current_time.toSec());
        // rviz_start_point(marker_pub_start_point, plan_start_msg);
        // plan_start_msg.x = vehicleState.x;
        // plan_start_msg.y = vehicleState.y;
        // cascsl(plan_start_msg.x, plan_start_msg.y, 0, 0, 0, 0, car_msg, 1);//计算车的sl等信息

        // Box_2d obs1({5, 0}, 0.0, 0.8, 0.5, 0);
        // Box_2d obs2({10, 0}, 0.0, 0.8, 0.5, 0);
        // Box_2d obs3({10, 1}, 0.0, 0.8, 0.5, 0);
        // Box_2d obs4({10, -1}, 0.0, 0.8, 0.5, 0);
        // vector<Box_2d> obstacles;
        // obstacles.push_back(obs1);
        // obstacles.push_back(obs2);
        // obstacles.push_back(obs3);
        // obstacles.push_back(obs4);
        // obs_deal(obstacles);
      
        // CalaStartCost(car_msg, obs_msg);
        
        // make_dp_path();
        // draw_dp_point_and_path();

        // geometry_msgs::Twist cmd_vel=get_cmd_vel(dp_pathxy.Point_msg_vec[0].x,dp_pathxy.Point_msg_vec[0].y,vehicleState.x,vehicleState.y,vehicleState.yaw);
        // pub_control_cmd.publish(cmd_vel);

        // rate.sleep();
     }
}
void em::bc(const ros::TimerEvent& event){
     static int        time_1 = 0;
     static int        time_100_flag = 0;
     time_1++;
    cout<<time_100_flag<<endl;
    if(time_1>=1000){
        time_1 = 0;
        time_100_flag++;
    }

}

void em::nodeStart(int argc, char **argv){
    ros::init(argc, argv, "em");
    ros::NodeHandle nc;

    marker_pub = nc.advertise<visualization_msgs::Marker>("point", 1);                              // 在rviz上显示录制的路径
    marker_pub_start_point = nc.advertise<visualization_msgs::Marker>("start_point", 1);            // 在rviz上显示录制的路径
    marker_pub_all_obs =nc.advertise<visualization_msgs::Marker>("all_obs", 1);                     // 所有障碍物
    marker_pub_dp_point=nc.advertise<visualization_msgs::Marker>("dp_point", 1);      
    marker_pub_dp_point_choose=nc.advertise<visualization_msgs::Marker>("dp_point_choose", 1);    

    pub_globle_path = nc.advertise<by_djstl::Path>("globle_path_by_two", 10);//发布全局规划路径
    pub_control_cmd = nc.advertise<geometry_msgs::Twist>("/cmd_vel", 10);//发布cmd_vel

    ros::Subscriber sub_globle_path = nc.subscribe("globle_path", 1, &em::globlepathGetCallBack, this); // 接受local规划路径
    // ros::Subscriber sub_odom = nc.subscribe("/odom", 1, &em::control_odometryGetCallBack, this);        // 控制节点
    
    
    // timer = nc.createTimer(ros::Duration(0.001),&em::bc,this,false,false);//定时器中断


    // ros::AsyncSpinner spinner(3);
    // spinner.start();
    // ros::waitForShutdown();

   ros::spin();

    ros::NodeHandle nh;

}


int main(int argc, char  *argv[])
{
    em node;
    node.nodeStart(argc,argv);
    return 0;
}
void em::draw_dp_point_and_path(){
    Eigen::MatrixXd a(row,col);
    Point_vec_msg draw_point_vec;
     for(int i=0;i<row;i++){
        for(int j=0;j<col;j++){
            float s = car_msg.Point_msg_vec[0].s + (j+1) * sample_s;
            float l =  ((row+1)/2-i-1) * sample_l;
            draw_point_vec.Point_msg_vec.push_back(Calaxy(s,l));
            a(i,j)=Calaxy(s,l).x;
        }
     }
     visualization_msgs::Marker points;
     points.header.frame_id = "odom";
     points.header.stamp = ros::Time::now();
     points.ns = "points_and_lines";
     points.action = visualization_msgs::Marker::ADD;
     points.pose.orientation.w = 1.0;
     points.id = 0;
     points.type = visualization_msgs::Marker::POINTS;
     points.scale.x = 0.1;
     points.scale.y = 0.1;
     points.color.g = 1.0f;
     points.color.a = 1.0;
     for(int i=0;i<draw_point_vec.Point_msg_vec.size();i++){
        geometry_msgs::Point p;
        p.x = draw_point_vec.Point_msg_vec[i].x;
        p.y = draw_point_vec.Point_msg_vec[i].y;
        p.z = 0;
        points.points.push_back(p);
     }
     marker_pub_dp_point.publish(points);
//////////////////////////////////////////////////////////
     visualization_msgs::Marker points1;
     points1.header.frame_id = "odom";
     points1.header.stamp = ros::Time::now();
     points1.ns = "points_and_lines";
     points1.action = visualization_msgs::Marker::ADD;
     points1.pose.orientation.w = 1.0;
     points1.id = 0;
     points1.type = visualization_msgs::Marker::POINTS;
     points1.scale.x = 0.5;
     points1.scale.y = 0.5;
     points1.color.r = 1.0f;
     points1.color.a = 1.0;
     for (int i = 0; i <  dp_pathxy.Point_msg_vec.size(); i++)
     {
        geometry_msgs::Point p;
        p.x = dp_pathxy.Point_msg_vec[i].x;
        p.y = dp_pathxy.Point_msg_vec[i].y;
        p.z = 0;
        points1.points.push_back(p);
     }
     marker_pub_dp_point_choose.publish(points1);
}
void em::make_dp_path(){
        for(int j=1;j<col;j++){
            for(int i=0;i<row;i++){
                float cur_node_s = plan_start_msg.x + (j + 1) * sample_s;
                float cur_node_l = ((row+1)/2 -(i+1))*sample_l;
                for(int k=0;k<row;k++){
                    float pre_node_s = plan_start_msg.x + (j) * sample_s;
                    float pre_node_l = ((row + 1) / 2 - (k + 1)) * sample_l;

                    float cost_Neighbour= CalcNeighbourCost(pre_node_s,pre_node_l,cur_node_s,cur_node_l);
                   // cost_node[k][j-1]=cost_Neighbour;
                    float pre_min_cost=  cost_node[k][j-1];
                    float cost_temp=cost_Neighbour+pre_min_cost;
                    if(cost_temp<cost_node[i][j]){
                        cost_node[i][j]=cost_temp;
                        pre_node_index[i][j]=k;
                    }
                }
            }
        }
        Eigen::MatrixXd aa(row,col);
        for(int i=0;i<row;i++){
            for(int j=0;j<col;j++){
                aa(i,j)=cost_node[i][j];
            }
        }
       // cout<<aa<<endl;

        int index=0;
        float min_cost=10000000000;
        for(int i=0;i<row;i++){
            if(cost_node[i][col-1]<min_cost){
                min_cost=cost_node[i][col-1];
                index=i;
            }
        }
        int cur_index=index;
        vector<int> dp_node_list_row;
        dp_node_list_row.resize(col);
        for(int i=0;i<col;i++){
            int pre_index = pre_node_index[cur_index][col - 1 - i];
            dp_node_list_row[col - i - 1] = cur_index;
            cur_index = pre_index;
        }
        vector<float> dp_path_s,dp_path_l;
        dp_path_s.resize(col),dp_path_l.resize(col);
        dp_path_s.clear();dp_path_l.clear();
        dp_pathxy.Point_msg_vec.clear();
        for(int i=0;i<col;i++){
            dp_path_s.push_back(car_msg.Point_msg_vec[0].s + (i+1) * sample_s);
            dp_path_l.push_back((row+1)/2-(dp_node_list_row[i]+1)*sample_l);
            Point_msg dp_pathxy_msg=Calaxy(dp_path_s[i],dp_path_l[i]);
            dp_pathxy.Point_msg_vec.push_back(dp_pathxy_msg);
        }
}
Point_msg em::Calaxy(float s,float l){
    int index=0;
     while(s>referenceline.s[index]){
        index++;
     }

     Eigen::MatrixXd match_point(2, 1);
     match_point << referenceline.x[index], referenceline.y[index];
     float match_point_heading = referenceline.heading[index];
     float match_point_kappa = referenceline.kappa[index];
     float ds = s - referenceline.s[index];

     Eigen::MatrixXd match_tor(2, 1);
     match_tor << cos(match_point_heading), sin(match_point_heading);
     Eigen::MatrixXd proj_point = match_point + ds * match_tor;
     float proj_heading = match_point_heading + ds * match_point_kappa;
     float proj_kappa = match_point_kappa;
     float proj_x = proj_point(0);
     float proj_y = proj_point(1);

     Eigen::MatrixXd nor(2, 1);
     nor << -sin(match_point_heading), cos(match_point_heading);
     Eigen::MatrixXd point=proj_point+l*nor;

     Point_msg trant_point;
     trant_point.x = point(0);
     trant_point.y = point(1);
     return trant_point;
}
float em::CalcNeighbourCost(float pre_node_s,float pre_node_l,float cur_node_s,float cur_node_l){
        float start_l = pre_node_l;
        float start_dl = 0;
        float start_ddl = 0;
        float end_l = cur_node_l;
        float end_dl =0;
        float end_ddl = 0;
        float start_s=pre_node_s;
        float end_s=cur_node_s; 
       Eigen::VectorXf c= CalcQuinticCoeffient( start_l,start_dl ,start_ddl, end_l, end_dl,end_ddl,start_s,end_s);
       float a0 = c[0];
       float a1 = c[1];
       float a2 = c[2];
       float a3 = c[3];
       float a4 = c[4];
       float a5 = c[5];

       vector<float> ds,l,dl,ddl,dddl,obs_square;
       float cost_smooth=0,cost_ref=0,cost_collision=0;
       for(int i=0;i<10;i++){
        ds.push_back(start_s+(i)*float((end_s-start_s)*sample_s)/10.0);
       }
       for(int i=0;i<ds.size();i++){
        l.push_back(a0 + a1 * ds[i] + a2 * pow(ds[i], 2) + a3 * pow(ds[i], 3) + a4 * pow(ds[i], 4) + a5 * pow(ds[i], 5));
        dl.push_back(a1 + 2 * a2 * ds[i] + 3 * a3 * pow(ds[i], 2) + 4 * a4 * pow(ds[i], 3) + 5 * a5 * pow(ds[i], 4));
        ddl.push_back(2 * a2 + 6 * a3 * ds[i] + 12 * a4 * pow(ds[i], 2) + 20 * a5 * pow(ds[i], 3));
        dddl.push_back(6 * a3 + 24 * a4 * ds[i] + 60 * a5 * pow(ds[i], 2));
       }
       for(int i=0;i<ds.size();i++){
        // if(abs(ddl[i])>0.5 || abs(atan(dl[i])>0.4*3.1415926)){
        //      cost_smooth+=100000;
        // }
         cost_smooth+=w_cost_smooth_dl*pow(dl[i],2)+w_cost_smooth_ddl*pow(ddl[i],2)+w_cost_smooth_dddl*pow(dddl[i],2);
         cost_ref+=w_cost_ref_2*pow(l[i],2);
       }
        for(int i=0;i<obs_msg.Point_msg_vec.size();i++){
            float cost_collision_once=0;
              for(int j=0;j<ds.size();j++){
             float dlon = obs_msg.Point_msg_vec[i].s - ds[j];
             float dlat = obs_msg.Point_msg_vec[i].l - l[j];
             obs_square.push_back(sqrt(pow(dlon,2)+pow(dlat,2)));
            }     
            float min=10000000;
            for(int b=0;b<obs_square.size();b++){
                if(min>obs_square[b]) {min=obs_square[b];}
            }
            if(min>2){
                cost_collision_once=0;
            }else if(min<2 && min>0.5){
                cost_collision_once=w_cost_collision*2000/pow(min,2);
            }else if(min<0.5){
                cost_collision_once=w_cost_collision*100000000;
            }
            cost_collision=cost_collision+cost_collision_once;
            range_precote(cost_collision,1000000000,0);
        } 
      float cost=cost_collision+cost_smooth+cost_ref;
      return cost;
 }
void em::CalaStartCost(Point_vec_msg car_msg,Point_vec_msg obs_msg){
    for(int x=1;x<=row;x++){
        float start_l = car_msg.Point_msg_vec[0].l;
        float start_dl = car_msg.Point_msg_vec[0].l_d;
        float start_ddl = car_msg.Point_msg_vec[0].l_d_d;
        float end_l = ((row+1)/2-x)*sample_l;
        float end_dl =0;
        float end_ddl = 0;
        float start_s=car_msg.Point_msg_vec[0].s;
        float end_s=car_msg.Point_msg_vec[0].s+sample_s; 
       Eigen::VectorXf c= CalcQuinticCoeffient( start_l,start_dl ,start_ddl, end_l, end_dl,end_ddl,start_s,end_s);
       float a0 = c[0];
       float a1 = c[1];
       float a2 = c[2];
       float a3 = c[3];
       float a4 = c[4];
       float a5 = c[5];

       vector<float> ds,l,dl,ddl,dddl,obs_square;
       float cost_smooth=0,cost_ref=0,cost_collision=0;
       for(int i=0;i<10;i++){
            ds.push_back(start_s+float(i*sample_s)/10.0);
       }
       for(int i=0;i<ds.size();i++){
            l.push_back(a0 + a1 * ds[i] + a2 * pow(ds[i], 2) + a3 * pow(ds[i], 3) + a4 * pow(ds[i], 4) + a5 * pow(ds[i], 5));
            dl.push_back(a1 + 2 * a2 * ds[i] + 3 * a3 * pow(ds[i], 2) + 4 * a4 * pow(ds[i], 3) + 5 * a5 * pow(ds[i], 4));
            ddl.push_back(2 * a2 + 6 * a3 * ds[i] + 12 * a4 * pow(ds[i], 2) + 20 * a5 * pow(ds[i], 3));
            dddl.push_back(6 * a3 + 24 * a4 * ds[i] + 60 * a5 * pow(ds[i], 2));
       }
       for(int i=0;i<ds.size();i++){
        // if(abs(ddl[i])>0.5 || abs(atan(dl[i])>0.4*3.1415926)){
        //     cost_smooth+=100000;
        // }
         cost_smooth+=w_cost_smooth_dl*pow(dl[i],2)+w_cost_smooth_ddl*pow(ddl[i],2)+w_cost_smooth_dddl*pow(dddl[i],2);
         cost_ref+=w_cost_ref_start*sqrt(pow(l[i],2));
       }
        for(int i=0;i<obs_msg.Point_msg_vec.size();i++){
            float cost_collision_once=0;
            obs_square.clear();
            for(int j=0;j<ds.size();j++){
             float dlon = obs_msg.Point_msg_vec[i].s - ds[j];
             float dlat = obs_msg.Point_msg_vec[i].l - l[j];
             obs_square.push_back(sqrt(pow(dlon,2)+pow(dlat,2)));
            }     
            float min=10000000;
            for(int b=0;b<obs_square.size();b++){
                if(min>obs_square[b]) {min=obs_square[b];}
            }

            if(min>2){
                cost_collision_once=0;
            }else if(min<2 && min>0.5){
                cost_collision_once=w_cost_collision*1500/pow(min,2);
            }else if(min<0.5){
                cost_collision_once=w_cost_collision*10000000;
            }
         
            cost_collision=cost_collision+cost_collision_once;
  
            range_precote(cost_collision,1000000000,0);
        } 
         cost_node[x-1][0]=cost_collision+cost_smooth+cost_ref;
        //  cout << "cost_collision" << cost_collision << endl;
        //  cout << "cost_smooth" << cost_smooth << endl;
        //  cout << "cost_ref" << cost_ref << endl;
        //  cout << "row  " << x << endl<<endl;
    }
  }
  Eigen::VectorXf em::CalcQuinticCoeffient(float start_l, float start_dl ,float start_ddl, float end_l, float end_dl, 
                               float end_ddl,float start_s, float end_s){
    float start_s2 = start_s * start_s;
    float start_s3 = start_s2 * start_s;
    float start_s4 = start_s3 * start_s;
    float start_s5 = start_s4 * start_s;
    float end_s2 = end_s * end_s;
    float end_s3 = end_s2 * end_s;
    float end_s4 = end_s3 * end_s;
    float end_s5 = end_s4 * end_s;

    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(6, 6);
    A << 1, start_s, start_s2, start_s3, start_s4, start_s5,
        0, 1, 2 * start_s, 3 * start_s2, 4 * start_s3, 5 * start_s4,
        0, 0, 2, 6 * start_s, 12 * start_s2, 20 * start_s3,
        1, end_s, end_s2, end_s3, end_s4, end_s5,
        0, 1, 2 * end_s, 3 * end_s2, 4 * end_s3, 5 * end_s4,
        0, 0, 2, 6 * end_s, 12 * end_s2, 20 * end_s3;
    Eigen::VectorXf B = Eigen::VectorXf::Zero(6, 1);
    B<<start_l,start_dl,start_ddl,end_l,end_dl,end_ddl;
    Eigen::VectorXf c_eigen = A.colPivHouseholderQr().solve(B);
    return c_eigen;
  }
void em::all_obs_rviz(ros::Publisher marker_pub, vector<Box_2d> vel_obs_info)
{
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "odom";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "points_and_lines";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id = 2;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.2;
    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;
    // Create the vertices for the points
    geometry_msgs::Point p;
    for (uint32_t i = 0; i < vel_obs_info.size(); i++)
    {
        for (uint32_t j = 0; j < vel_obs_info[i].Box2d_corner.size(); j++)
        {
            double y = vel_obs_info[i].Box2d_corner[j].y;
            double x = vel_obs_info[i].Box2d_corner[j].x;
            float z = 0;
            geometry_msgs::Point p;
            p.x = x;
            p.y = y;
            p.z = z;
            line_list.points.push_back(p);
        }
    }
    marker_pub.publish(line_list);
}
void em::obs_deal(vector<Box_2d> obstacles){
    std::vector<Box_2d> obstacles_;
    for (int i = 0; i < obstacles.size(); i++)
    {
        Box_2d obs = obstacles[i];
        MatrixXd vector_obs(2, 1);
        vector_obs << obstacles[i].center_x() - vehicleState.x, obstacles[i].center_y() - vehicleState.y;
        MatrixXd tor(2, 1), nor(2, 1);
        tor << cos(vehicleState.yaw), sin(vehicleState.yaw);
        nor << -sin(vehicleState.yaw), cos(vehicleState.yaw);

        float long_distace = (vector_obs.transpose() * tor)(0);
        float lat_distace  = (vector_obs.transpose() * nor)(0);

        if (long_distace < 60 && long_distace > -10 && lat_distace > -10 && lat_distace < 10)
        {
            obstacles_.push_back(obs);
            cascsl(obstacles[i].center_x(), obstacles[i].center_y(), 0, 0, 0, 0, obs_msg, obstacles.size());
        }
    }
  all_obs_rviz(marker_pub_all_obs, obstacles_);
}
void em::cascsl(float set_x,float set_y,float set_vx,float set_vy,float set_ax,float set_ay,Point_vec_msg& point_vec_msg,int max_size){
    int index = 0;
    float min_dis = 10000;
    for (int i = 0; i < referenceline.x.size(); i++)
    {
        float dis = pow(referenceline.x[i] - set_x, 2) + pow(referenceline.y[i] -set_y, 2);
        if (dis < min_dis)
        {
            min_dis = dis;
            index = i;
        }
    }
    MatrixXd vector_match_point(2, 1);
     vector_match_point << referenceline.x[index], referenceline.y[index];
    MatrixXd vector_match_point_direction(2, 1);
    vector_match_point_direction << cos(referenceline.heading[index]), sin(referenceline.heading[index]);

    MatrixXd vector_r(2, 1);
    vector_r << set_x,set_y;

    MatrixXd vector_d(1, 2);
    vector_d = vector_r - vector_match_point;

    MatrixXd ds = vector_d.transpose() * vector_match_point_direction;
    float d_by = ds(0);
    MatrixXd vector_proj_poin = vector_match_point + d_by * vector_match_point_direction;
    Path_msg paht_msg_proj_; // 投影点

    paht_msg_proj_.x.push_back(vector_proj_poin(0));
    paht_msg_proj_.y.push_back(vector_proj_poin(1));
    paht_msg_proj_.heading.push_back(referenceline.heading[index] + d_by * referenceline.kappa[index]);
    paht_msg_proj_.kappa.push_back(referenceline.kappa[index]);

    int n = referenceline.x.size();
    float S0 = 0;
    float L0 = 0;
    MatrixXd index2s_1 = Eigen::MatrixXd::Zero(n, 1);
    for (int i = 0; i < n; i++)
    {
        if(i==0){ index2s_1(i)=0;}
 else{ index2s_1(i) =index2s_1(i-1)+ sqrt(pow(referenceline.x[i] - referenceline.x[i - 1], 2) + pow(paht_msg_final.y[i] - paht_msg_final.y[i - 1], 2));}
    }
    float s_temp = index2s_1(index);
    MatrixXd vector_match_2_origin(2,1), vector_match_2_origin_next(2,1);
    vector_match_2_origin << (paht_msg_proj_.x[0] - referenceline.x[index]), (paht_msg_proj_.y[0] - referenceline.y[index]);
  if(index<referenceline.x.size()-1){
    vector_match_2_origin_next << referenceline.x[index + 1] - referenceline.x[index],
                                  referenceline.y[index + 1] - referenceline.y[index];
  }else{
    vector_match_2_origin_next << referenceline.x[index ] - referenceline.x[index-1],
                                  referenceline.y[index ] - referenceline.y[index-1];
  }
    vector_match_2_origin_next << referenceline.x[index + 1] - referenceline.x[index],
        referenceline.y[index + 1] - referenceline.y[index];

       if((vector_match_2_origin.transpose()*vector_match_2_origin_next)(0)>0){
            S0=s_temp+sqrt((vector_match_2_origin.transpose()*vector_match_2_origin)(0));
        }else{
            S0=s_temp-sqrt((vector_match_2_origin.transpose()*vector_match_2_origin)(0));
        }///这里面的S0是从优化参考线处开始的
          MatrixXd n_r(2,1),r_h(2,1),r_r(2,1);
          n_r<<-sin( paht_msg_proj_.heading[0]),cos( paht_msg_proj_.heading[0]);
          r_h<<set_x,set_y;
          r_r<<paht_msg_proj_.x[0],  paht_msg_proj_.y[0];
          L0=((r_h-r_r).transpose()*n_r)(0);
      
    //////////////////////////////////////////////////////////////////////////
    MatrixXd v_h(2,1),n_r_d(2,1),t_r_d(2,1);
    v_h<<set_vx,set_vy;
    n_r_d<<-sin(paht_msg_proj_.heading[0]),cos( paht_msg_proj_.heading[0]);
    t_r_d<<cos( paht_msg_proj_.heading[0]),sin( paht_msg_proj_.heading[0]);

    float L0_dot=(v_h.transpose()*n_r_d)(0);
    float S0_dot=(v_h.transpose()*t_r_d)(0)/(1-paht_msg_proj_.kappa[0]*L0);
    float dl_set=0;
    if(abs(S0_dot)<0.000001){dl_set=0;}
    else                    {dl_set=L0_dot/S0_dot;}
    ////////////////////////////////////////////////////////////////////////////////////
    MatrixXd a_h(2,1),n_r_dd(2,1),t_r_dd(2,1);
    a_h << set_ax, set_ay;
    n_r_dd << -sin(paht_msg_proj_.heading[0]), cos(paht_msg_proj_.heading[0]);
    t_r_dd << cos(paht_msg_proj_.heading[0]), sin(paht_msg_proj_.heading[0]);
    float L0_dot_dot=(a_h.transpose()*n_r_dd)(0)-paht_msg_proj_.kappa[0]*L0*S0_dot*S0_dot;
    float S0_dot_dot=(1/(1-paht_msg_proj_.kappa[0]*L0))*((a_h.transpose()*t_r_dd)(0)+2*paht_msg_proj_.kappa[0]*dl_set*S0_dot*S0_dot);
    float ddl_set=0;
    if(S0_dot_dot<0.000001) {ddl_set=0;}
    else                    {ddl_set=(L0_dot_dot-dl_set*S0_dot_dot)/(S0_dot_dot*S0_dot_dot);}
    
    if( point_vec_msg.Point_msg_vec.size()>=max_size){
        point_vec_msg.Point_msg_vec.clear();
    }
    Point_msg a;
    a.x = set_x;
    a.y = set_y;
    a.vx = set_vx;
    a.vy = set_vy;
    a.ax = set_ax;
    a.ay = set_ay;
    a.s=S0;
    a.l=L0;
    a.s_d = S0_dot;
    a.l_d = L0_dot;
    a.s_d_d = S0_dot_dot;
    a.l_d_d = L0_dot_dot;
    a.dl=dl_set;
    a.ddl=ddl_set;
    point_vec_msg.Point_msg_vec.push_back(a);
 }
void  em::rviz_start_point(ros::Publisher marker_pub, Point_msg plan_start_msg){
    visualization_msgs::Marker points;
    points.header.frame_id = "odom";
    points.header.stamp = ros::Time::now();
    points.ns = "points_and_lines";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = 0.1;
    points.scale.y = 0.1;
    points.color.r = 1.0f;
    points.color.a = 1.0;
    geometry_msgs::Point p;
    p.x = plan_start_msg.x;
    p.y = plan_start_msg.y;
    p.z = 0;
    points.points.push_back(p);
    marker_pub.publish(points);
}
 void em::find_plan_start_point(double time_now){//找到规划起点，并把起点前面的轨迹存入？母鸡
    if(1){//find_start_point_first_flag==0
        find_start_point_first_flag=1;
        plan_start_msg.x = vehicleState.x;
        plan_start_msg.y = vehicleState.y;
        plan_start_msg.heading=vehicleState.yaw;
        plan_start_msg.kappa=0;
        plan_start_msg.vx=0;
        plan_start_msg.vy=0;
        plan_start_msg.ax=0;
        plan_start_msg.ay=0;
        plan_start_msg.time=current_time.toSec()+0.1;
    }else{
        float lon_err=0;
        float lat_err=0;
        float x_cur = vehicleState.x;
        float y_cur = vehicleState.y;
        float heading_cur= vehicleState.yaw;
        float kappa_cur=0;

        float vx_cur = vehicleState.vx * cos(heading_cur) - vehicleState.vy * sin(heading_cur);
        float vy_cur = vehicleState.vx * sin(heading_cur) + vehicleState.vy * cos(heading_cur);
        float ax_cur = vehicleState.ax * cos(heading_cur) - vehicleState.ay * sin(heading_cur);
        float ay_cur = vehicleState.ax * sin(heading_cur) + vehicleState.ay * cos(heading_cur);

        float dt=0.1;
        for(int i=0;i<paln_msg.Point_msg_vec.size();i++){
            if(current_time.toSec()>=paln_msg.Point_msg_vec[i].time  && current_time.toSec()<paln_msg.Point_msg_vec[i+1].time){
                float pre_x_desire = paln_msg.Point_msg_vec[i].x;
                float pre_y_desire = paln_msg.Point_msg_vec[i].y;
                float pre_heading_desire = paln_msg.Point_msg_vec[i].heading;
                Matrix2d tor,nor,d_err;
                tor<<cos(pre_heading_desire),sin(pre_heading_desire);
                nor<<-sin(pre_heading_desire),cos(pre_heading_desire);
                d_err<<vehicleState.x-pre_x_desire,vehicleState.y-pre_y_desire;
                  lon_err = abs((d_err.transpose() * tor)(0));
                  lat_err = abs((d_err.transpose() * nor)(0));
                break;
            }
        }
        if(lon_err>2.5 || lat_err>0.5){
            plan_start_msg.x = x_cur + vx_cur * dt + 0.5 * ax_cur * dt * dt;
            plan_start_msg.y = y_cur + vy_cur * dt + 0.5 * ay_cur * dt * dt;
            plan_start_msg.vx = vx_cur + ax_cur * dt;
            plan_start_msg.vy = vy_cur + ay_cur * dt;
            plan_start_msg.heading=atan2(plan_start_msg.vy,plan_start_msg.vx);
            plan_start_msg.ax = ax_cur;
            plan_start_msg.ay = ay_cur;
            plan_start_msg.kappa=kappa_cur;
            plan_start_msg.time=current_time.toSec()+0.1;
            return ;
        }else{
            for (int i = 0; i < paln_msg.Point_msg_vec.size(); i++)
            {
                if (current_time.toSec() >= paln_msg.Point_msg_vec[i].time && current_time.toSec() < paln_msg.Point_msg_vec[i + 1].time)
                {
                    plan_start_msg.x = paln_msg.Point_msg_vec[i].x;
                    plan_start_msg.y = paln_msg.Point_msg_vec[i].y;
                    plan_start_msg.heading = paln_msg.Point_msg_vec[i].heading;
                    plan_start_msg.kappa = paln_msg.Point_msg_vec[i].kappa;
                    plan_start_msg.vx = paln_msg.Point_msg_vec[i].v * cos(plan_start_msg.heading);
                    plan_start_msg.vy = paln_msg.Point_msg_vec[i].v * sin(plan_start_msg.heading);
                    Matrix2d tor, nor, a_tor, a_nor;
                    tor << cos(plan_start_msg.heading), sin(plan_start_msg.heading);
                    nor << -sin(plan_start_msg.heading), cos(plan_start_msg.heading);
                    a_tor = paln_msg.Point_msg_vec[i].a * tor;
                    a_nor = paln_msg.Point_msg_vec[i].v * paln_msg.Point_msg_vec[i].v * plan_start_msg.kappa * nor;
                    plan_start_msg.ax = a_tor(0) + a_nor(0);
                    plan_start_msg.ay = a_tor(1) + a_nor(1);
                    plan_start_msg.time = paln_msg.Point_msg_vec[i].time;
                    
                    Point_vec_msg stitch;
                    if(i>=19){
                        for(int j=0;j<20;j++){
                            stitch.Point_msg_vec.push_back(paln_msg.Point_msg_vec[j-20+i]);
                        }
                    }else{
                          for(int j=0;j<i;j++){
                            stitch.Point_msg_vec.push_back(paln_msg.Point_msg_vec[j]);
                        }
                    }
                }
            }
        }
    }
 }
////////////////////////////////////////////////////////////////////////////////////////////////////////
void  rviz_road(ros::Publisher marker_pub, Point point){
    visualization_msgs::Marker points;
    points.header.frame_id = "odom";
    points.header.stamp = ros::Time::now();
    points.ns = "points_and_lines";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = 0.1;
    points.scale.y = 0.1;
    points.color.b = 1.0f;
    points.color.a = 1.0;

    geometry_msgs::Point p;
    p.x = point.x;
    p.y = point.y;
    p.z = 0;
    points.points.push_back(p);

    marker_pub.publish(points);
}
void  rviz_road_1(ros::Publisher marker_pub, Path_msg paht_msg){
    visualization_msgs::Marker points;
    points.header.frame_id = "odom";
    points.header.stamp = ros::Time::now();
    points.ns = "points_and_lines";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = 0.3;
    points.scale.y = 0.3;
    points.color.g = 1.0f;
    points.color.a = 1.0;
    for(int i=0;i< paht_msg.x.size();i++ ){
        geometry_msgs::Point p;
        p.x = paht_msg.x[i];
        p.y = paht_msg.y[i];
        p.z = 0;
        points.points.push_back(p);
    }
    marker_pub.publish(points);
}
 void em::osqp_referenceline_by()
 {
    int n = paht_msg_referenceline_init.x.size();
    A1 = Eigen::MatrixXd::Zero(2 * n - 4, 2 * n);
    A2 = Eigen::MatrixXd::Zero(2 * n - 2, 2 * n);
    A3 = Eigen::MatrixXd::Identity(2 * n, 2 * n);
    f = Eigen::MatrixXd::Zero(2 * n, 1);
    lb = Eigen::MatrixXd::Zero(2 * n, 1);
    ub = Eigen::MatrixXd::Zero(2 * n, 1);

    for (int i = 1; i <= n; i++)
    {
        f(2 * i - 2) = paht_msg_referenceline_init.x[i - 1];
        f(2 * i - 1) = paht_msg_referenceline_init.y[i - 1];
        lb(2 * i - 2) = f(2 * i - 2) - x_lb;
        ub(2 * i - 2) = f(2 * i - 2) + x_ub;
        lb(2 * i - 1) = f(2 * i - 1) - y_lb;
        ub(2 * i - 1) = f(2 * i - 1) + y_ub;
    }
    for (int i = 1; i <= 2 * n - 5; i++)
    {
        A1(i - 1, i - 1) = 1;
        A1(i - 1, i + 1) = -2;
        A1(i - 1, i + 3) = 1;
        A1(i, i) = 1;
        A1(i, i + 2) = -2;
        A1(i, i + 4) = 1;
    }
    for (int i = 1; i <= 2 * n - 3; i++)
    {
        A2(i - 1, i - 1) = 1;
        A2(i - 1, i + 1) = -1;
        A2(i, i) = 1;
        A2(i, i + 2) = -1;
    }

    H = 2 * (w_cost_smooth * (A1.transpose() * A1) + w_cost_length * (A2.transpose() * A2) + w_cost_ref * A3);

    real_t H_real[H.rows() * H.cols()];
    real_t A_real[H.rows() * H.cols()];
    for (int i = 0; i < H.rows(); i++)
    {
        for (int j = 0; j < H.cols(); j++)
        {
            H_real[i * H.cols() + j] = H(i, j);
            A_real[i * H.cols() + j] = 1;
        }
    }
    real_t g[2 * paht_msg_referenceline_init.x.size()];
    real_t lb_real[2 * paht_msg_referenceline_init.x.size()];
    real_t ub_real[2 * paht_msg_referenceline_init.x.size()];
    for (int i = 0; i < 2 * (paht_msg_referenceline_init.x.size()); i++)
    {
        g[i] = -2 * w_cost_ref * f(i);
        lb_real[i] = lb(i);
        ub_real[i] = ub(i);
    }
    int_t nWSR = 1000;
    QProblem example(H.rows(), H.cols());
    Options options;
    example.setOptions(options);
    example.init(H_real, g, A_real, lb_real, ub_real, NULL, NULL, nWSR, NULL);
    real_t xOpt[2 * (paht_msg_referenceline_init.x.size())];
    paht_msg_final.x.clear();
    paht_msg_final.y.clear();
    example.getPrimalSolution(xOpt);
    vector<float> wx_refrenceline,wy_refrenceline;
    for (int i = 0; i < (paht_msg_referenceline_init.x.size()) - 1; i++)
    {
        paht_msg_final.x.push_back(xOpt[i * 2]);
        paht_msg_final.y.push_back(xOpt[i * 2 + 1]);
        wx_refrenceline.push_back(xOpt[i * 2]);
        wy_refrenceline.push_back(xOpt[i * 2+1]);
    }

    rviz_road_1(marker_pub, paht_msg_final); // 可视化二次规划完成的参考线
    get_headingandkappa(wx_refrenceline,wy_refrenceline,referenceline);
    referenceline.s.clear();
    for(int i=0;i<referenceline.x.size();i++){
        if(i==0){ referenceline.s.push_back(0);}
        else{
            referenceline.s.push_back(referenceline.s[i-1]+sqrt(pow(referenceline.x[i]-referenceline.x[i-1],2)+
                                       pow(referenceline.y[i]-referenceline.y[i-1],2)));    
        }
    }
 }
void em::globlepathGetCallBack( by_djstl::Path msg){
   ros::Rate rate(1);
   wx_by.clear();
   wy_by.clear();
     for(int i=0;i<msg.points.size();i++){
         Point p;
         p.x = msg.points[i].x;
         p.y = msg.points[i].y;
         wx_by.push_back(p.x);
         wy_by.push_back(p.y);
     }
     get_headingandkappa(wx_by, wy_by, paht_msg);
     get_gloab_point_flag = 1;

     index_nearcar = get_indexandmsg(paht_msg, vehicleState); // 找到匹配点
     get_touying_point();                                     // 找到投影点
     find_road_scope(wx_by.size()-10 , 9);                    // 取投影点前后部分
     osqp_referenceline_by();                                 // 参考线二次规划,

     by_djstl::Path globle_path;
     for (int i = 0; i < referenceline.x.size(); i++)
     {

         by_djstl::PathPoint p;
         p.heading=referenceline.heading[i];
         p.kappa=referenceline.kappa[i];
         p.s=referenceline.s[i];
         p.x = paht_msg_final.x[i];
         p.y = paht_msg_final.y[i];
         globle_path.points.push_back(p);
     }
     cout<<"pub over"<<endl;
     this->pub_globle_path.publish(globle_path);

     rate.sleep();

 }
void em::get_headingandkappa(vector<float> wx_by,vector<float> wy_by,Path_msg& save_msg){
     save_msg.heading.clear();
     save_msg.x.clear();
     save_msg.y.clear();
     save_msg.kappa.clear();
     save_msg.s.clear();

     vector<float> heading;
     vector<float> kappa;
     vector<float> ds;
     vector<float> dx_vec, dy_vec, dx_vec_final, dy_vec_final, ds_by;
     vector<float> dheading, dheading_final;
     for (int i = 0; i < wx_by.size(); i++)
     {
        float dx=wx_by[i+1]-wx_by[i];
        float dy=wy_by[i+1]-wy_by[i];
        ds_by.push_back(sqrt(pow(dx,2)+pow(dy,2)));
        dx_vec.push_back(dx);
        dy_vec.push_back(dy);
    }
     for(int i=0;i<wx_by.size();i++){
      if(i==0 || i==wx_by.size()-1){
            dx_vec_final.push_back(dx_vec[i]);
            dy_vec_final.push_back(dy_vec[i]);
        }else {
            dx_vec_final.push_back((dx_vec[i - 1] + dx_vec[i + 1]) / 2);
            dy_vec_final.push_back((dy_vec[i - 1] + dy_vec[i + 1]) / 2);
        }
         save_msg.x.push_back(wx_by[i]);
         save_msg.y.push_back(wy_by[i]);
         save_msg.heading.push_back(atan2(dy_vec_final[i], dx_vec_final[i]));
     }
     for(int i=0;i<wx_by.size();i++){
         float dheading1 = save_msg.heading[i + 1] - save_msg.heading[i];
         dheading.push_back(dheading1);
     }
      for(int i=0;i<wx_by.size();i++){ 
         if (i == 0 || i == wx_by.size() - 1)
         {
             dheading_final.push_back( dheading[i]);
         }
         else
         {
             dheading_final.push_back((dheading[i -1 ] + dheading[i+1])/2);
         }
         float kappa=sin(dheading_final[i])/ds_by[i];
         save_msg.kappa.push_back(kappa);
      }

      for(int i=0;i<4;i++){
         save_msg.kappa.pop_back();
         save_msg.heading.pop_back();
      }
      for (int i = 0; i < 4; i++)
      {
         float kappa = save_msg.kappa[save_msg.kappa.size() - 1];
         float heading = save_msg.heading[save_msg.heading.size() - 1];
         save_msg.kappa.push_back(kappa);
         save_msg.heading.push_back(heading);
      }
}
// void em::get_headingandkappa(vector<float> wx_by,vector<float> wy_by,Path_msg& save_msg){
//      save_msg.heading.clear();
//      save_msg.x.clear();
//      save_msg.y.clear();
//      save_msg.kappa.clear();
//      save_msg.s.clear();
//      for (int i = 0; i < wx_by.size(); i++)
//      {
//          save_msg.x.push_back(wx_by[i]);
//          save_msg.y.push_back(wy_by[i]);
//     }
//    for(int i=1;i<wx_by.size();i++){
//          float dis1 = sqrt(pow(wx_by[i - 1] - wx_by[i], 2) + pow(wy_by[i - 1] - wy_by[i], 2));
//          float dis2 = sqrt(pow(wx_by[i - 1] - wx_by[i+1], 2) + pow(wy_by[i - 1] - wy_by[i+1], 2));
//          float dis3 = sqrt(pow(wx_by[i] - wx_by[i+1], 2) + pow(wy_by[i] - wy_by[i+1], 2));
//          float dis=dis1*dis1 + dis3*dis3 - dis2*dis2;
//          float cosA = dis / (2 * dis1 * dis3); // 余弦定理求角度
//          float sinA = sqrt(1 - cosA * cosA);   // 求正弦
//          float curvity = 0.5 * dis2 / sinA;    // 正弦定理求外接圆半径
//          float kappa = 1 / curvity;          // 半径的倒数是曲率，半径越小曲率越大
//          if(save_msg.kappa.empty()){
//             save_msg.kappa.push_back(kappa);
//             save_msg.kappa.push_back(kappa);
//          }else{
//             save_msg.kappa.push_back(kappa);
//          }
//    }
//     for(int i=0;i<wx_by.size()-1;i++){
//          float heading = 0;
//          float pi = 3.14;
//          float dx = wx_by[i + 1] - wx_by[i];
//          float dy = wy_by[i + 1] - wy_by[i];
//          if(dx == 0){
//             heading=pi/2;
//             if (dy == 0)
//             {
//                 heading = 0;
//             }
//             else if (wy_by[i + 1] < wy_by[i])
//             {
//                 heading = 3.0 * pi / 2.0;
//             }
//          }
//          else if(wx_by[i+1]>wx_by[i] &&  wy_by[i+1]> wy_by[i]){
//               heading =atan(dy / dx);
//          }
//          else if(wx_by[i+1]<wx_by[i] &&  wy_by[i+1]> wy_by[i]){
//               heading =pi/2+atan(-dy / dx);
//          }
//          else if(wx_by[i+1]<wx_by[i] &&  wy_by[i+1]< wy_by[i]){
//               heading =pi+atan(dy / dx);
//          }
//          else if(wx_by[i+1]>wx_by[i] &&  wy_by[i+1]< wy_by[i]){
//               heading =3*pi/2+atan(dy / -dx);
//          }
//          if(i==wx_by.size()-2){
//               save_msg.heading.push_back(heading);
//               save_msg.heading.push_back(heading);
//          }else{
//              save_msg.heading.push_back(heading);
//          }      
//    }
// }
void em::find_road_scope(int pre,int beh){
    int start_index=-1;
    if((index_nearcar-beh)<1){
        start_index=0;
    }else if((index_nearcar+pre)>paht_msg.x.size()){
        start_index=paht_msg.x.size()-pre-beh-1;
    }else{
        start_index=index_nearcar-beh-1;
    }
    paht_msg_referenceline_init.heading.clear();
    paht_msg_referenceline_init.x.clear();
    paht_msg_referenceline_init.y.clear();
    paht_msg_referenceline_init.kappa.clear();
    paht_msg_referenceline_init.s.clear();
    for(int i=start_index;i<=start_index+pre+beh;i++){
        paht_msg_referenceline_init.x.push_back(paht_msg.x[i]);
        paht_msg_referenceline_init.y.push_back(paht_msg.y[i]);
        paht_msg_referenceline_init.heading.push_back(paht_msg.heading[i]);
        paht_msg_referenceline_init.kappa.push_back(paht_msg.kappa[i]);
    }
     rviz_road_1(marker_pub,paht_msg_referenceline_init);//可视化截取的参考线部分
}
void em::get_touying_point(){
  MatrixXd vector_match_point(2,1);
  vector_match_point<<paht_msg.x[index_nearcar],paht_msg.y[index_nearcar];

  MatrixXd vector_match_point_direction(2,1);
  vector_match_point_direction<<cos(paht_msg.heading[index_nearcar]),sin(paht_msg.heading[index_nearcar]);

  MatrixXd vector_r(2,1);
  vector_r<<vehicleState.x,vehicleState.y;

  MatrixXd vector_d(1,2);
  vector_d=vector_r-vector_match_point;

  MatrixXd ds=vector_d.transpose() *vector_match_point_direction;
  float d_by=ds(0);
  MatrixXd vector_proj_poin=vector_match_point+d_by*vector_match_point_direction;
  paht_msg_proj.x.push_back(vector_proj_poin(0));
  paht_msg_proj.y.push_back(vector_proj_poin(1));
  paht_msg_proj.heading.push_back(paht_msg.heading[index_nearcar]+d_by*paht_msg.kappa[index_nearcar]);
  paht_msg_proj.kappa.push_back(paht_msg.kappa[index_nearcar]);
}
 int em::get_indexandmsg(Path_msg paht_msg,State vehicleState ){
    int index=0;
    float min_dis=10000;
    for(int i=0;i<paht_msg.x.size();i++){
        float dis=pow(paht_msg.x[i]-vehicleState.x,2)+pow(paht_msg.y[i]-vehicleState.y,2);
        if(dis<min_dis){
            min_dis=dis;
            index=i;
        }
    }
    return index;
 }