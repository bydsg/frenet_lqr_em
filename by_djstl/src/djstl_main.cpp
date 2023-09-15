#include "include/headfile.h"
using namespace std;


//加载txt中路径
 std::vector<Road> load_direction_Road(const std::string fileName)
{ 
    std::vector<Road> roads;
    bool record_points = false;
    Road road;
    std::string line;
    std::string line3;
    std::ifstream fs;
    fs.open(fileName, std::ios::in);
    while (getline(fs, line)) {
      
        if (line.length() > 0) {
            std::stringstream ss(line);
            std::stringstream sss(line);
            std::string line2;
            int id;
            ss>>line2;
            if (line2 == "road")
            {   
                ss>>id;
                road.id = id;
                continue;
            }
        
            if (line2 == "point")
            {
                record_points = true;
                continue;
            }
            if (line2 != "pre" && record_points)
            {
             Point  point ;
             point.l=2;
             point.r=2;  
             ss >> point.x >> point.y>>point.theta;
            //  point.x+=17.866777;
            //  point.y+=33.367218;
            //  point.theta+=1.551813;
             road.road_points.push_back(point); 
            }
    
            if (line2 == "pre")
            {
                int count = 0;
                while(getline(sss,line3,' ')){ 
                   count++;
                   if (count>=2)
                   {
                       std::stringstream ssss(line3);
                       int id;
                       ssss>>id;
                       road.pre.push_back(id);
                   }
                }
                record_points = false;
                continue;
            }
 
            if (line2 == "beh")
            {    
                int count = 0;
                while(getline(sss,line3,' ')){ 
                   count++;
                   if (count>=2)
                   {
                       std::stringstream ssss(line3);
                       int id;
                       ssss>>id;
                       road.beh.push_back(id);
                   }
                }
                
                record_points = false;
                roads.push_back(road); 
                road = {0};               
                continue;  
            }
        }   
    }
    
    fs.close();
    return roads;
}
class globle_planning
{
public:
    ros::Publisher marker_pub;
    ros::Publisher pub_globle_path;
    ros::Publisher globle_path_marker_pub;
    ros::Subscriber sub_odom;
    ros::Subscriber sub_end_odom;
    vector<Road> roads;
    vector<Road> roads_new;
    Point vehicle_pose;

    void nodeStart(int argc, char **argv);
    void odometryGetCallBack(const nav_msgs::Odometry::ConstPtr odometry_msg);
    void rviz_road(ros::Publisher marker_pub, std::vector<Road> roads);
    void end_odom_callback(const geometry_msgs::PoseStamped::ConstPtr msg);
    void rviz_road2(ros::Publisher marker_pub, by_djstl::Path globle_path);
    void odometryGetCallBack_GPS(const geometry_msgs::PoseStamped::ConstPtr odometry_msg);
    void low_pass_filter_road(vector<Road> &roads, int num, float max_dis, int road_num);
};
void globle_planning::low_pass_filter_road(vector<Road> &roads,int num,float max_dis,int road_num){
    vector<Point> road_points_new;
    Point p_last;
    p_last.x = roads[road_num].road_points[0].x;
    p_last.y = roads[road_num].road_points[0].y;
    //滤掉大于max_dis m的飘的点
    int count_dis=0;
    for(int i=0;i<roads[road_num].road_points.size();i++){
        Point p;
      
        p.x = roads[road_num].road_points[i].x;
        p.y = roads[road_num].road_points[i].y;
        float dis=sqrt(pow(p.x-p_last.x,2)+pow(p.y-p_last.y,2));
        cout<<dis<<endl;
        if(dis<max_dis){
            road_points_new.push_back(p);
            p_last.x = p.x;
            p_last.y = p.y;
            count_dis=0;
        }else{
            if(count_dis<5){
             count_dis++;  
            }else{
             p_last.x = p.x;
             p_last.y = p.y;
            }           
        }
    }

    int count=0;
    float x_sum=0,y_sum=0;
    vector<Point> road_points_new_new;

    road_points_new_new.push_back(road_points_new[0]);
    road_points_new_new.push_back(road_points_new[1]);
    road_points_new_new.push_back(road_points_new[2]);
    for(int i=3;i<road_points_new.size()-3;i++){//前3,后3个点不绿
        if(count<num){
            x_sum += road_points_new[i].x;
            y_sum += road_points_new[i].y;
            count++;
        }else{
            Point p;
            p.x = x_sum / (float)count;
            p.y = y_sum / (float)count;
            y_sum = 0;
            x_sum = 0;
            road_points_new_new.push_back(p);
            count = 0;
        }
      
    }
    road_points_new_new.push_back(road_points_new[road_points_new.size() - 3]);
    road_points_new_new.push_back(road_points_new[road_points_new.size() - 2]);
    road_points_new_new.push_back(road_points_new[road_points_new.size() - 1]);
    Road road;
    for (int i = 0; i < road_points_new_new.size(); i++)
    {
        Point p;
        p.x = road_points_new_new[i].x;
        p.y = road_points_new_new[i].y;
        road.road_points.push_back(p);
    }
    int id;
    std::vector<int> pre;
    std::vector<int> beh;
    for(int i=0;i<roads[road_num].beh.size();i++){
        int beh =roads[road_num].beh[i];
        road.beh.push_back(beh);
    }  
    for(int i=0;i<roads[road_num].pre.size();i++){
        int pre =roads[road_num].pre[i];
        road.pre.push_back(pre);
    }
    road.id=roads[road_num].id;

    std::vector<Road>::iterator it = roads.begin() + road_num;
    *it = road;
}
void globle_planning::odometryGetCallBack_GPS(const  geometry_msgs::PoseStamped::ConstPtr odometry_msg){
// 得到车辆的定位信息和速度信息
    ros::Rate loop_rate(10);
    rviz_road(this->marker_pub, this->roads);  
    
    this->vehicle_pose.x = odometry_msg->pose.position.x;
    this->vehicle_pose.y = odometry_msg->pose.position.y ;
    double raw, pitch, theta;
     tf::Quaternion q;
     tf::quaternionMsgToTF(odometry_msg->pose.orientation, q);
     tf::Matrix3x3(q).getRPY(raw, pitch, theta);
    this->vehicle_pose.theta = theta;
    loop_rate.sleep();
}
void globle_planning::odometryGetCallBack(const nav_msgs::Odometry::ConstPtr odometry_msg){
// 得到车辆的定位信息和速度信息
    ros::Rate loop_rate(10);
     rviz_road(this->marker_pub, this->roads);  

    double raw, pitch, theta;
    tf::Quaternion q;
    tf::quaternionMsgToTF(odometry_msg->pose.pose.orientation, q);
    tf::Matrix3x3(q).getRPY(raw, pitch, theta);
    this->vehicle_pose.theta = theta;
    this->vehicle_pose.x = odometry_msg->pose.pose.position.x;
    this->vehicle_pose.y = odometry_msg->pose.pose.position.y;
    loop_rate.sleep();
}
void globle_planning:: rviz_road(ros::Publisher marker_pub, std::vector<Road> roads){
    visualization_msgs::Marker points;
    points.header.frame_id = "odom";
    points.header.stamp = ros::Time::now();
    points.ns = "points_and_lines";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = 0.4;
    points.scale.y = 0.4;
    points.color.r = 1.0f;
    points.color.a = 1.0;

    for (size_t i = 0; i < roads.size(); i++)
    {
        for (size_t j = 0; j < roads[i].road_points.size(); j++)
        {

            geometry_msgs::Point p;
            p.x = roads[i].road_points[j].x;
            p.y = roads[i].road_points[j].y;
            p.z = 0;
            points.points.push_back(p);
        }
    }
    marker_pub.publish(points);
}
void globle_planning::rviz_road2(ros::Publisher marker_pub, by_djstl::Path globle_path){
    visualization_msgs::Marker points;
    points.header.frame_id =  "odom";
    points.header.stamp = ros::Time::now();
    points.ns = "points_and_lines";
    points.action =  visualization_msgs::Marker::ADD;
    points.pose.orientation.w =  1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::LINE_STRIP;
    points.scale.x = 0.01;
    points.scale.y = 0.01;
    points.color.r = 1.0f;
    points.color.a = 1.0;
    for (size_t i = 0; i < globle_path.points.size(); i++)
    {  
             geometry_msgs::Point p;
             p.x =  globle_path.points[i].x ;
             p.y =  globle_path.points[i].y;
             p.z = 0;
             points.points.push_back(p);        
    }
    marker_pub.publish(points);
}
void globle_planning::end_odom_callback(const geometry_msgs::PoseStamped::ConstPtr msg ){
    ros::Rate loop_rate(10);
    Point end_point;
    end_point.x = msg->pose.position.x;
    end_point.y = msg->pose.position.y;
    double raw, pitch, theta;
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.orientation, q);
    tf::Matrix3x3(q).getRPY(raw, pitch, theta);
    end_point.theta = theta;

    cout << " end_point.x " << end_point.x << endl;
    cout << " end_point.y " << end_point.y << endl;
    cout << " end_point.theta " << end_point.theta << endl;

    //加载结构地图
    int dist[maxSize], path[maxSize], v, E;
    Graphlnk<int, int>  G; // 声明图对象
    G.inputGraph(this->roads);     // 创建图
    //G.outputGraph();
    int u0 = 10;
    int e0 = 10;
    int e_nearest = 0;
     /////获取车辆当前位置对应的路径id
    double min_distance1 =100000;
    int u1=0;
    int turn_flag=0;
    for (size_t i = 0; i < this->roads.size(); i++)
    {
        for (size_t j = 0; j < this->roads[i].road_points.size(); j++)
        {
            double distance = std::sqrt(std::pow(this->vehicle_pose.x-this->roads[i].road_points[j].x, 2) + std::pow(this->vehicle_pose.y-this->roads[i].road_points[j].y, 2));
            if(distance <= min_distance1){
                u0 = 1+i;
                min_distance1 = distance;
                u1=j;
            }
        }
    }
    double min_distance2 =10000000000;
    for (size_t i = 0; i < this->roads.size(); i++)
    {
        for (size_t j = 0; j < this->roads[i].road_points.size(); j++)
        {
            double distance =(end_point.x-this->roads[i].road_points[j].x)*(end_point.x-this->roads[i].road_points[j].x)+ (end_point.y-this->roads[i].road_points[j].y)*(end_point.y-this->roads[i].road_points[j].y);
            //double distance = std::sqrt(std::pow(end_point.x-this->roads[i].road_points[j].x, 2) + std::pow(end_point.y-this->roads[i].road_points[j].y, 2));
            if(distance <= min_distance2){
                e0 = 1+i;
                e_nearest = j;
                min_distance2 = distance;
            }
        }
    }   

    if(e_nearest<u1){
        turn_flag=1;
        int temp=u1;
        u1=e_nearest;
        e_nearest=temp;
    }
    v = G.getVertexPos(u0); // 取得起始顶点的位置
    E = G.getVertexPos(e0); // 取得起始end的位置
    Dijkstra(G, v, dist, path); // 调用Dijkstra函数
    std::vector<int> road_id;
    road_id = printShortestPath(G, v, E, dist, path); // 输出到各个顶点的最短路径

    int e_id = e0 - 1;                                // 终点道路所在的下标

    std::vector<Point> begin_end_point;
    std::vector<Point> path_points;
    begin_end_point.push_back(this->roads[e_id].road_points[e_nearest]);
    Point p_next;
    Point p_forward;

    path_points = B_spline_optimization(begin_end_point);

     ////整合路径，并将路径信息转换成msg信息
      by_djstl::Path globle_path;
      for (size_t i = 0; i < road_id.size(); i++)
      {
        if (i != road_id.size() - 1)
        {
            for (size_t j = 0; j < roads[road_id[i] - 1].road_points.size(); j++)
            {
                by_djstl::PathPoint p;
                p.x = roads[road_id[i] - 1].road_points[j].x;
                p.y = roads[road_id[i] - 1].road_points[j].y;
                globle_path.points.push_back(p);
            }
        }
        else
        {
           
            for (size_t j = u1; j <= e_nearest; j++)
            {
                by_djstl::PathPoint p;
                p.x = roads[road_id[i] - 1].road_points[j].x;
                p.y = roads[road_id[i] - 1].road_points[j].y;
                globle_path.points.push_back(p);
            }
        }
      }
      if(turn_flag){
        turn_flag=0;
        reverse(globle_path.points.begin(),globle_path.points.end());
      }

    rviz_road2(globle_path_marker_pub, globle_path);
    this->pub_globle_path.publish(globle_path);
    loop_rate.sleep();
}
void globle_planning::nodeStart(int argc, char **argv)
{
    ros::init(argc, argv, "globle_planning");
    ros::NodeHandle nc;

    string roadMap_path = "/home/by/code/lqr_by2/data/1.txt"; // 地图录制位置///home/lwh/Lpp_files/data/road_path8.txt
    this->roads = load_direction_Road(roadMap_path);

      low_pass_filter_road(roads,10,8,0);
      B_spline_optimization_road(roads,0,3);

    //   FILE *fp_s;
    //   fp_s = fopen("/home/by/code/lqr_by2/data/4_chazhi.txt", "a");
    //   for (int i = 0; i < roads[0].road_points.size(); i++)
    //   {
    //       fprintf(fp_s, "%c %lf %lf %lf", 'c', roads[0].road_points[i].x, roads[0].road_points[i].x, 0); // Angle
    //       fprintf(fp_s, "\r\n");
    //   }
    //   fclose(fp_s);

      this->pub_globle_path = nc.advertise<by_djstl::Path>("globle_path", 10);                             // 发布全局规划路径
      this->marker_pub = nc.advertise<visualization_msgs::Marker>("road_rviz", 1);                         // 在rviz上显示录制的路径
      this->globle_path_marker_pub = nc.advertise<visualization_msgs::Marker>("road_rviz_globle_path", 1); // 显示规划的全局路径

      // 订阅相关节点
      this->sub_odom = nc.subscribe("/odom", 1, &globle_planning::odometryGetCallBack, this); // 控制节点
      // this->sub_odom = nc.subscribe("/rear_post", 1, &globle_planning::odometryGetCallBack_GPS, this);//控制节点gps
      this->sub_end_odom = nc.subscribe("/move_base_simple/goal", 1, &globle_planning::end_odom_callback, this); // 订阅终点位置

      ros::spin();
}


int main(int argc, char  *argv[])
{
    globle_planning node;
    node.nodeStart(argc, argv);
    return(0);
}
