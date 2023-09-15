#ifndef _RVIZ_H_
#define _RVIZ_H_

#include "headfile.h"
using namespace cpprobotics;

void  rviz_road(ros::Publisher marker_pub,FrenetPath final_path){
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
    points.color.r = 1.0f;
    points.color.a = 1.0;

    for (size_t i = 0; i < final_path.x.size(); i++)
    { 
            geometry_msgs::Point p;
            p.x = final_path.x[i];
            p.y = final_path.y[i];
            p.z = 0;
            points.points.push_back(p);
    }
    marker_pub.publish(points);
}

void all_obs_rviz(ros::Publisher marker_pub, vector<Box_2d> vel_obs_info)
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
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;
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

void rviz_veh_box2d(ros::Publisher marker_pub, Box_2d veh_info){
   visualization_msgs::Marker points;
    points.header.frame_id = "odom";
    points.header.stamp = ros::Time::now();
    points.ns = "points_and_lines";
    points.action =  visualization_msgs::Marker::ADD;
    points.pose.orientation.w =  1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;

    points.scale.x = 5;
    points.scale.y = 5;
    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

     visualization_msgs::Marker line_list1;
     line_list1.header.frame_id = "odom";
    line_list1.header.stamp = ros::Time::now();
     line_list1.ns = "points_and_lines";
    line_list1.action = visualization_msgs::Marker::ADD;
    line_list1.pose.orientation.w = 1.0;
    line_list1.id = 2;
    line_list1.type = visualization_msgs::Marker::LINE_LIST;
    line_list1.scale.x = 0.1;
    // Line list is blue
    line_list1.color.b = 1.0;
    line_list1.color.a = 1.0;
         for (uint32_t j= 0; j<veh_info.Box2d_corner.size() ;j++ )
            {    double y = veh_info.Box2d_corner[j].y;
                double  x =veh_info.Box2d_corner[j].x;
                float z = 0;
                geometry_msgs::Point p;
                      p.x = x;
                      p.y = y;
                     p.z = z;
                    line_list1.points.push_back(p);
         }    
    marker_pub.publish(line_list1);
  }

#endif