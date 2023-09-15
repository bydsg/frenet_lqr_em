#include "frenet.h"
float last_x=0;
float last_y=0;
float c_speed = 30.0 / 3.6;
float c_d = 0;
float c_d_d = 0;
float c_d_dd = 0;
float s0 = 0;
void lattice::control_odometryGetCallBack(const nav_msgs::Odometry::ConstPtr odometry_msg){
    ros::Rate rate(1);
     ///////////////////障碍物
  // std::vector<Poi_f> obstcles{{{10.24, -0.7},{20.5, 2.21},}};
    std::vector<Poi_f> obstcles{{{10.24, -0.7}}};
    Box_2d obs1({10.24, -0.7}, 0.0, 0.8, 0.5, 0);
   // Box_2d obs2({20.5, 2.21}, 0.0, 0.8, 0.5, 0);
    obstacles_.clear();
    obstacles_.push_back(obs1);
   // obstacles_.push_back(obs2);
    all_obs_rviz(marker_pub_all_obs, obstacles_);
    ////////////////////////////

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

    Box_2d  vel_info({ vehicleState.x  , vehicleState.y},vehicleState.yaw, 2.0, 1.0, 0);
    rviz_veh_box2d(this->veh_pub, vel_info );

    if(first_flag){
          cascsl(vehicleState.x, vehicleState.y, vehicleState.vx,   vehicleState.vy, vehicleState.ax, vehicleState.ax, car_msg, 1);//计算车的sl等信息
          if (!la_first_flag)
          {
            c_speed = 30.0 / 3.6;
            c_d = car_msg.Point_msg_vec[0].l;
            c_d_d = 0;
            c_d_dd = 0;
            s0 = car_msg.Point_msg_vec[0].s;
            la_first_flag=1;
          } 
            FrenetPath final_path = frenet_optimal_planning(referenceline, s0, c_speed, c_d, c_d_d, c_d_dd, obstcles);
            cout<<final_path.s.size()<<endl;
            float dis_min=10000;
            int index=0;
            for(int i=0;i<final_path.x.size();i++ )  {
              float dis=sqrt(pow(final_path.x[i]-vehicleState.x,2)+pow(final_path.y[i]-vehicleState.y,2));
              if(dis<dis_min){
                index=i;
                dis_min=dis;
              }
            }
            s0 = final_path.s[index];
            c_d = final_path.d[index];
            c_d_d = final_path.d_d[index];
            c_d_dd = final_path.d_dd[index];
            c_speed = final_path.s_d[index];

            path_la.points.clear();
            for (int i = 0; i < final_path.s.size(); i++)
            {
              la::path_la_point p;
              p.x = final_path.x[i];
              p.y = final_path.y[i];
              p.s = final_path.s[i];
              p.s_dot = final_path.s_d[i];
              p.s_dot_dot = final_path.d_dd[i];
              p.s_dot_dot_dot = final_path.d_ddd[i];
              p.d = final_path.d[i];
              p.d_dot = final_path.d_d[i];
              p.d_dot_dot = final_path.d_dd[i];
              p.d_dot_dot = final_path.d_ddd[i];
              p.heading = final_path.yaw[i];
              path_la.points.push_back(p);
            }
            last_x = final_path.x.back();
            last_y = final_path.y.back();
            marker_pub_la2control.publish(path_la);
            rviz_road(marker_pub, final_path);
            }
            // rate.sleep();
            // if (std::pow((final_path.x[1] - referenceline.x.back()), 2) + std::pow((final_path.y[1] - referenceline.y.back()), 2) <= 1.0)
            // {
            //   while (1)
            //   {
            //     cout<<"over"<<endl;
            //   }
            // }


 }
void lattice::globlepathGetCallBack(const by_djstl::Path msg){
    ros::Rate rate(10);
    referenceline.heading.clear();
    referenceline.kappa.clear();
    referenceline.x.clear();
    referenceline.y.clear();
    referenceline.s.clear();
    for (int i = 0; i < msg.points.size(); i++)
    {
        referenceline.heading.push_back(msg.points[i].heading);
        referenceline.kappa.push_back(msg.points[i].kappa);
        referenceline.s.push_back(msg.points[i].s);
        referenceline.x.push_back( msg.points[i].x);
        referenceline.y.push_back(msg.points[i].y);
   }
   //保存二次规划发过来的路径
      FILE *fp_s;
      fp_s = fopen("/home/by/code/lqr_by2/data/4_em.txt","a");
   for(int i=0;i<referenceline.x.size();i++){
        fprintf(fp_s, "%c %lf %lf %lf", 'c', referenceline.x[i], referenceline.y[i], 0); // Angle
        fprintf(fp_s, "\r\n");
   }
     fclose(fp_s);
   first_flag=1;
   cout<<"get em_two global_path"<<endl;
}
void lattice::nodeStart(int argc, char **argv){
    ros::init(argc, argv, "la");
    ros::NodeHandle nc;

    ros::Subscriber sub_globle_path = nc.subscribe("globle_path_by_two", 1, &lattice::globlepathGetCallBack, this); // 接受local规划路径
    ros::Subscriber sub_odom = nc.subscribe("/odom", 1, &lattice::control_odometryGetCallBack, this);        // 控制节点

    marker_pub_all_obs=nc.advertise<visualization_msgs::Marker>("all_obs_la", 1);    
    marker_pub = nc.advertise<visualization_msgs::Marker>("la", 1);
    marker_pub_la2control=nc.advertise<la::path_la>("la2control",1);
    veh_pub = nc.advertise<visualization_msgs::Marker>("car", 1);//显示车
    ros::spin();
}

int main(int argc, char  *argv[])
{
  lattice node;
  node.nodeStart(argc,argv);
  return 0;
}
