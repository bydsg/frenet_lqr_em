#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

#define TARGET_SPEED 0.3
#define slow_angle   0.4

void range_precote(auto &val, float max, float min)
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

 geometry_msgs::Twist get_cmd_vel(float goal_x,float goal_y,float start_x,float start_y,float yaw){
    double alpha = atan2(goal_y - start_y, goal_x - start_x) - yaw; // 误差角度
    if(alpha<-3.14) {alpha=alpha+6.28;}
    if(alpha>3.14) {alpha=alpha-6.28;}

     float wheelAngle = alpha;
    
     float speed = 0 ; // 调整车辆速度
     if(abs(wheelAngle)>slow_angle)    {speed = TARGET_SPEED*slow_angle/abs(alpha) ;}
     else                              { speed = TARGET_SPEED;}
     range_precote(speed,0.3,0);
     geometry_msgs::Twist cmd_vel;
     // 发布前轮转角
     cmd_vel.linear.x = speed;
     cmd_vel.linear.y = 0.0;

     cmd_vel.angular.z = wheelAngle;
     return cmd_vel;
 }

#endif
