#ifndef frenet_h
#define frenet_h
#include "include/headfile.h"

#define SIM_LOOP 500
#define MAX_SPEED  50.0 / 3.6  // maximum speed [m/s]
#define MAX_ACCEL  2.0  // maximum acceleration [m/ss]
#define MAX_CURVATURE  1.0  // maximum curvature [1/m]
#define MAX_ROAD_WIDTH  7.0  // maximum road width [m]
#define D_ROAD_W  1.0  // road width sampling length [m]
#define DT  0.2  // time tick [s]
#define MAXT  5.0  // max prediction time [m]
#define MINT  1.0  // min prediction time [m]
#define TARGET_SPEED  30.0 / 3.6  // target speed [m/s]
#define D_T_S  5.0 / 3.6  // target speed sampling length [m/s]
#define N_S_SAMPLE  1  // sampling number of target speed
#define ROBOT_RADIUS  2  // robot radius [m]

#define KJ  0.1
#define KT  0.1
#define KD  1.0
#define KLAT  1.0
#define KLON  1.0

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
class lattice{
public:
  ros::Publisher marker_pub;
  ros::Publisher marker_pub_la2control;
  ros::Publisher marker_pub_all_obs;
  ros::Publisher veh_pub;

  std::vector<Box_2d> obstacles_;
  by_djstl::Path globle_path;
  la::path_la path_la;
  Path_msg referenceline;
  State vehicleState;
  Point_vec_msg car_msg;

  bool la_first_flag=0;
  bool first_flag=0;

  void nodeStart(int argc, char **argv);
  void globlepathGetCallBack(const by_djstl::Path msg);
  void control_odometryGetCallBack(const nav_msgs::Odometry::ConstPtr odometry_msg);
  void cascsl(float set_x,float set_y,float set_vx,float set_vy,float set_ax,float set_ay,Point_vec_msg& point_vec_msg,int max_size);
};

float sum_of_power(std::vector<float> value_list){
  float sum = 0;
  for(float item:value_list){
    sum += item*item;
  }
  return sum;
};

Vec_Path calc_frenet_paths(
    float c_speed, float c_d, float c_d_d, float c_d_dd, float s0){
  std::vector<FrenetPath> fp_list;
  for(float di=-1*MAX_ROAD_WIDTH; di<MAX_ROAD_WIDTH; di+=D_ROAD_W){
    for(float Ti=MINT; Ti<MAXT; Ti+=DT){
      FrenetPath fp;
      QuinticPolynomial lat_qp(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti);
      for(float t=0; t<Ti; t+=DT){
        fp.t.push_back(t);
        fp.d.push_back(lat_qp.calc_point(t));
        fp.d_d.push_back(lat_qp.calc_first_derivative(t));
        fp.d_dd.push_back(lat_qp.calc_second_derivative(t));
        fp.d_ddd.push_back(lat_qp.calc_third_derivative(t));
      }
      for(float tv=TARGET_SPEED - D_T_S * N_S_SAMPLE;
          tv < TARGET_SPEED + D_T_S * N_S_SAMPLE;
          tv+=D_T_S){

        FrenetPath fp_bot = fp;
        QuarticPolynomial lon_qp(s0, c_speed, 0.0, tv, 0.0, Ti);

        fp_bot.max_speed = std::numeric_limits<float>::min();
        fp_bot.max_accel = std::numeric_limits<float>::min();
        for(float t_:fp.t){
          fp_bot.s.push_back(lon_qp.calc_point(t_));
          fp_bot.s_d.push_back(lon_qp.calc_first_derivative(t_));
          fp_bot.s_dd.push_back(lon_qp.calc_second_derivative(t_));
          fp_bot.s_ddd.push_back(lon_qp.calc_third_derivative(t_));
          if(fp_bot.s_d.back() > fp_bot.max_speed){
            fp_bot.max_speed = fp_bot.s_d.back();
          }
          if(fp_bot.s_dd.back() > fp_bot.max_accel){
            fp_bot.max_accel = fp_bot.s_dd.back();
          }
        }

        float Jp =0;// sum_of_power(fp.d_ddd);
        float Js =0;// sum_of_power(fp_bot.s_ddd);
        float ds =0;// (TARGET_SPEED - fp_bot.s_d.back());

        fp_bot.cd = KJ * Jp + KT * Ti + KD * std::pow(fp_bot.d.back(), 2);
        fp_bot.cv = KJ * Js + KT * Ti + KD * ds;
        fp_bot.cf = KLAT * fp_bot.cd + KLON * fp_bot.cv+sum_of_power(fp.d);

        fp_list.push_back(fp_bot);
      }
    }
  }
  return fp_list;
};

void calc_global_paths(Vec_Path & path_list, Path_msg referenceline){
  for (Vec_Path::iterator path_p=path_list.begin(); path_p!=path_list.end();path_p++){
    for(unsigned int i=0; i<path_p->s.size(); i++){
      if (path_p->s[i] >= referenceline.s.back()){
        break;
      }
      int index=0;
      float s=path_p->s[i];
      float l=path_p->d[i];
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

      path_p->x.push_back(point(0));
      path_p->y.push_back(point(1));
    }

    for(int i=0; i<path_p->x.size()-1; i++){
      float dx = path_p->x[i + 1] - path_p->x[i];
      float dy = path_p->y[i + 1] - path_p->y[i];
      path_p->yaw.push_back(std::atan2(dy, dx));
      path_p->ds.push_back(std::sqrt(dx * dx + dy * dy));
    }

    path_p->yaw.push_back(path_p->yaw.back());
    path_p->ds.push_back(path_p->ds.back());


    path_p->max_curvature = std::numeric_limits<float>::min();
    for(int i=0; i<path_p->x.size()-1; i++){
      path_p->c.push_back((path_p->yaw[i+1]-path_p->yaw[i])/path_p->ds[i]);
      if(path_p->c.back() > path_p->max_curvature){
        path_p->max_curvature = path_p->c.back();
      }
    }
  }
};

bool check_collision(FrenetPath path, const Vec_Poi ob){
  for(auto point:ob){
    for(unsigned int i=0; i<path.x.size(); i++){
      float dist = std::pow((path.x[i] - point[0]), 2) + std::pow((path.y[i] - point[1]), 2);
      if (dist <= ROBOT_RADIUS * ROBOT_RADIUS){
        return false;
      }
    }
  }
  return true;
};

Vec_Path check_paths(Vec_Path path_list, const Vec_Poi ob){
	Vec_Path output_fp_list;//path.max_speed < MAX_SPEED && path.max_accel < MAX_ACCEL && path.max_curvature < MAX_CURVATURE && 
  for(FrenetPath path:path_list){
    if (check_collision(path, ob)){
      output_fp_list.push_back(path);
    }
  }
  return output_fp_list;
};
FrenetPath frenet_optimal_planning(
    Path_msg referenceline, float s0, float c_speed,
    float c_d, float c_d_d, float c_d_dd, Vec_Poi ob){
  Vec_Path fp_list = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0);
  calc_global_paths(fp_list, referenceline);
  Vec_Path save_paths = check_paths(fp_list, ob);

  float min_cost = std::numeric_limits<float>::max();
  FrenetPath final_path;
  for(auto path:save_paths){
    if (min_cost >= path.cf){
      min_cost = path.cf;
      final_path = path;
    }
  }
  return final_path;
};   

void lattice::cascsl(float set_x,float set_y,float set_vx,float set_vy,float set_ax,float set_ay,Point_vec_msg& point_vec_msg,int max_size){
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
 else{ index2s_1(i) =index2s_1(i-1)+ sqrt(pow(referenceline.x[i] - referenceline.x[i - 1], 2) + pow(referenceline.y[i] - referenceline.y[i - 1], 2));}
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
#endif