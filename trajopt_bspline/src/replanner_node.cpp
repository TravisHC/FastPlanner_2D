#include <string>
#include <iostream>
#include <fstream>
#include <math.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
#include <time.h>
#include <random>
#include <memory>

// #include <ros/ros.h>
// #include <ros/console.h>
// #include <nav_msgs/Odometry.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/Point.h>
// #include <visualization_msgs/MarkerArray.h>
// #include <visualization_msgs/Marker.h>

#include <quadrotor_msgs/Bspline.h>
// #include <quadrotor_msgs/PositionCommand.h>
// #include <quadrotor_msgs/ReplanCheck.h>
// #include <quadrotor_msgs/SpatialTemporalTrajectory.h>

#include <grad_replanner/backward.hpp>
#include <grad_replanner/bezier_base.h>

#include "grad_replanner/grad_band_optimizer.h"
#include "grad_replanner/non_uniform_bspline.h"
#include "grad_replanner/bspline_replanner.h"
#include "../matplotlib-cpp/matplotlibcpp.h"

using namespace std;
using namespace Eigen;

namespace plt = matplotlibcpp;//可视化

Eigen::MatrixXd ctrl_pts_Vis;


namespace backward {
backward::SignalHandling sh;
}

#define INIT_TRAJ_ID 0
#define OPT_TRAJ_ID 1
#define INIT_PT_ID 2
#define OPT_PT_ID 3


// simulation param from launch file
double _MAX_Vel, _MAX_Acc;
double _time_emergency;
double _esdf_grid_freq, _vis_traj_width;
string _frame_id;

// useful global variables
bool _has_odom   = false;
bool _on_replan  = false;

double _replan_traj_time, _time_extra;
double _t_bspline_cmd_start, _t_bspline_cmd_end;

// ros::Time _replan_final_time = ros::TIME_MIN;
// ros::Time _replan_start_time = ros::TIME_MAX;

int _replan_traj_id = 1;
shared_ptr<SDFMap> _sdf_map;
unique_ptr<BsplineReplanner> _bspline_replanner;
int succ_replan_num_ = 0;

double _traj_resolution;
vector<Vector3d> _traj_pts;
void gradLocalReplan(const vector<Vector3d> & traj_pts, const double & time_interval, const MatrixXd & vel, const MatrixXd & acc, double local_to_global_time);
void BsplineFeasibleCheck(NonUniformBspline traj, bool & is_feas, bool & is_safe, Eigen::Vector3d & collision_pt);

// void displayPathWithColor(vector<Eigen::Vector3d> path, Eigen::Vector4d color, int id);
void drawBspline(NonUniformBspline bspline, int id, Eigen::Vector4d color, bool show_ctrl_pts,
                  double size2, int id2, Eigen::Vector4d color2);

// void visCheckTrajPoints(const vector<Vector3d> & traj_pts);
// void visBsplineInitialization(const MatrixXd & ctrl_pts);
// void visFillMinima(Eigen::Vector3d center, Eigen::Vector3d sc);

//ros::Time _replan_request_time;
quadrotor_msgs::Time _replan_request_time;
double _replan_time_length;

bool is_first_request = false;
int request_cnt = 0;
#if 0
void rcvReplanRequestCallBack()
{   
    if( !_sdf_map->hasDepthObservation() ) 
        return;
    
    if(request_cnt == 0)
        time_1st_request = ros::Time::now();

    request_cnt ++;

    ros::Time t0 = ros::Time::now();
    _replan_request_time = replan_request_msg.header.stamp;
    _replan_time_length  = replan_request_msg.replan_time_length;

    double plan_time_interval  = replan_request_msg.plan_points_time_interval;
    double check_time_interval = replan_request_msg.check_points_time_interval;
    
    MatrixXd acc(2, 3), vel(2, 3);

    Vector3d start_vel(replan_request_msg.start_velocity.x,     replan_request_msg.start_velocity.y,     replan_request_msg.start_velocity.z);
    Vector3d target_vel(replan_request_msg.stop_velocity.x,     replan_request_msg.stop_velocity.y,      replan_request_msg.stop_velocity.z);

    Vector3d start_acc(replan_request_msg.start_acceleration.x, replan_request_msg.start_acceleration.y, replan_request_msg.start_acceleration.z);
    Vector3d target_acc(replan_request_msg.stop_acceleration.x, replan_request_msg.stop_acceleration.y,  replan_request_msg.stop_acceleration.z);

    vel.row(0) = start_vel;
    vel.row(1) = target_vel;

    acc.row(0) = start_acc;
    acc.row(1) = target_acc;

    Vector3d check_pt;

    std::vector<Vector3d> traj_pts;            
    for(auto pt: replan_request_msg.plan_points){   
        Vector3d plan_pt(pt.x, pt.y, pt.z);
        traj_pts.push_back(plan_pt);    
    }
    visCheckTrajPoints(traj_pts);
	
    ros::Time t1 = ros::Time::now();
    //cout<<"time vis: ="<<(t1-t0).toSec()<<endl;
    Eigen::Vector3d grad;
    Eigen::Vector3d pt_1(replan_request_msg.check_points[0].x,replan_request_msg.check_points[0].y,replan_request_msg.check_points[0].z);
    for(int i = 0; i < replan_request_msg.check_points.size(); i++)
    {   
        check_pt << replan_request_msg.check_points[i].x, replan_request_msg.check_points[i].y, replan_request_msg.check_points[i].z;
    
        if( (check_pt-pt_1).norm() > 3.0 ) break;	
	   
	    double dis;
        dis = _sdf_map->getDistWithGradTrilinear(check_pt,  grad );
        if( dis <= 0.05  ){	   
            ROS_WARN("[Local Replanner] Collision detected, neeed replanning, dis:= %f", dis ); 
            if( i * check_time_interval < _time_emergency){
                ROS_ERROR("[Local Replanner] Emergency braking");
                
                quadrotor_msgs::Bspline stop_traj;
                stop_traj.start_time = _replan_request_time;
                stop_traj.traj_id    = _replan_traj_id ++;
                stop_traj.action = quadrotor_msgs::Bspline::ACTION_WARN_IMPOSSIBLE;
                _bspline_pub.publish(stop_traj);
                return;
            }
            else{   
                ros::Time t1 = ros::Time::now();
                gradLocalReplan(traj_pts, plan_time_interval, vel, acc, replan_request_msg.replan_to_global_time);
                //cout<<"time in local replannning := "<<(ros::Time::now() - t1).toSec()<<endl;
                return;
            }
        }
    }
}
#endif
quadrotor_msgs::Bspline getBsplineTraj(NonUniformBspline & traj_opt)
{   
    quadrotor_msgs::Bspline bspline;
    bspline.order = 3;

    bspline.start_time = _replan_request_time;
    bspline.traj_id    = _replan_traj_id ++;

    Eigen::MatrixXd ctrl_pts = traj_opt.getControlPoint();
    for (int i = 0; i < ctrl_pts.rows(); ++i) {
        Eigen::Vector3d pvt = ctrl_pts.row(i);
        geometry_msgs::Point pt;
        pt.x = pvt(0);
        pt.y = pvt(1);
        pt.z = pvt(2);
        bspline.pts.push_back(pt);
    }
    Eigen::VectorXd knots = traj_opt.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
        bspline.knots.push_back(knots(i));
    }

    return bspline;
}

void gradLocalReplan( const vector<Vector3d> & traj_pts, const double & time_interval, const MatrixXd & vel, const MatrixXd & acc , double local_to_global_time)
{
    /* generate sample points set on a naive trajectory*/
    vector<Eigen::Vector3d> start_end_derivative;
    start_end_derivative.push_back(vel.row(0));
    start_end_derivative.push_back(vel.row(1));
    start_end_derivative.push_back(acc.row(0));
    start_end_derivative.push_back(acc.row(1));

    Eigen::MatrixXd pos_v(2, 3);
    Eigen::MatrixXd vel_v = vel;
    Eigen::MatrixXd acc_v = acc;

    pos_v.row(0) = traj_pts.front();
    pos_v.row(1) = traj_pts.back();

    for(int i = 0;i < traj_pts.size();i++)
    {
        std::cout << "traj:" << std::endl;
        std::cout << traj_pts[i] << std::endl;
    }

    // std::cout << "time_interval =" << time_interval << std::endl;
    // std::cout << "vel =" << vel << std::endl;
    // std::cout << "acc =" << acc << std::endl;
    // std::cout << "local_to_global_time =" << local_to_global_time << std::endl;
   // ros::Time t1, t2;
   // t1 = ros::Time::now();    
    /* ========================================================================== */
    _bspline_replanner->setInput(_sdf_map, time_interval, traj_pts, start_end_derivative);
    _bspline_replanner->resetLambda2();
    bool traj_feas = false;
    bool traj_safe = false;
    Eigen:Vector3d collision_pt;
    NonUniformBspline traj_opt;

    Eigen::MatrixXd ctrl_pts;
    NonUniformBspline::BsplineParameterize(time_interval, traj_pts,
                                         start_end_derivative, ctrl_pts);
   // visBsplineInitialization(ctrl_pts);
   ctrl_pts_Vis = ctrl_pts;

   #if 1
    ctrl_pts_Vis = ctrl_pts;
    int iter = 0;
    while (iter < 5)
    {
        if (traj_feas && traj_safe) 
            break;
       
       std::cout << "start OPtimize " << std::endl;
      /* ---------- call replanning ---------- */
        
        _bspline_replanner->optimize(true);
        traj_opt = _bspline_replanner->getTraj();
        BsplineFeasibleCheck(traj_opt, traj_feas, traj_safe, collision_pt);

      /* ---------- fill local minima; enlarge lambda2 ---------- */
      if (!traj_safe){
        _bspline_replanner->renewLambda2(0.05);
      }

        iter++;
    }
    
    /* publish optimized B-spline */
    std::cout << "iter =" << iter << std::endl;
    traj_safe = true;
    if( traj_safe ){ 
        quadrotor_msgs::Bspline safe_spline = getBsplineTraj(traj_opt);
        double t_bspline_cmd_start, t_bspline_cmd_end;
        traj_opt.getRegion(t_bspline_cmd_start, t_bspline_cmd_end);

        double replan_traj_time = t_bspline_cmd_end - t_bspline_cmd_start;
        double time_extra = replan_traj_time - _replan_time_length;

        safe_spline.time_extra = time_extra;
        safe_spline.replan_to_global_time = local_to_global_time;

       // _bspline_pub.publish(safe_spline);
        _on_replan = true;

        drawBspline(traj_opt, OPT_TRAJ_ID + succ_replan_num_, Eigen::Vector4d(0, 1, 0, 1), true,
          0.1, OPT_PT_ID + succ_replan_num_, Eigen::Vector4d(1, 1, 0, 1));
        succ_replan_num_ += 3;
    }
    else{
        // do nothing
        cout<<"no safe replan, try it in next loop"<<endl;
    }

    #endif
}

void BsplineFeasibleCheck(NonUniformBspline traj, bool & is_feas, bool & is_safe, Eigen::Vector3d & collision_pt)
{   
    Vector3d pos, vel, acc;
    double t_bspline_start, t_bspline_end, t_duration;
    NonUniformBspline vel_traj = traj.getDerivative();  
    NonUniformBspline acc_traj = vel_traj.getDerivative();  

    traj.getRegion(t_bspline_start, t_bspline_end);   
    t_duration = t_bspline_end - t_bspline_start;

    is_safe = true;
    is_feas = true;
    collision_pt.setZero();
	
    Eigen::Vector3d grad;
    double dis;
    for(double t = 0.0; t < t_duration; t += 0.05)
    {
        pos =     traj.evaluateDeBoor(t_bspline_start + t);
        vel = vel_traj.evaluateDeBoor(t_bspline_start + t);
        acc = acc_traj.evaluateDeBoor(t_bspline_start + t);

        for(int i = 0; i < 3; i++){
            if( fabs(vel(i)) > _MAX_Vel || fabs(acc(i)) > _MAX_Acc )
                is_feas = false;
        }
	
        dis = _sdf_map->getDistWithGradTrilinear(pos,  grad );
        //std::cout << "dis =" << dis<< "pos =" << pos  << std::endl;
	   if( dis <  0.075 ){
            is_safe = false;
            if(collision_pt.norm() < 1e-5) collision_pt = pos;
        }
    }

}
#if 0
void visualizeESDFGrid(const ros::TimerEvent& event)
{   
    if( !_sdf_map->hasDepthObservation() ) 
        return;

    _sdf_map->publishMap();
    _sdf_map->publishMapInflate();
    _sdf_map->publishUpdateRange();
    _sdf_map->publishESDF();
}
#endif

std::vector<double> local_plan_x;
std::vector<double> local_plan_y;

int main(int argc, char** argv)
{
    
    _time_emergency = 1.0;
    _MAX_Vel = 1.0;
    _MAX_Acc = 1.0;
    _esdf_grid_freq = 1.0;
    _vis_traj_width = 0.15;

    _bspline_replanner.reset(new BsplineReplanner());
    _bspline_replanner->initialize(_MAX_Vel, _MAX_Acc);


    _sdf_map.reset(new SDFMap);

    double resolution = 0.15;
    double x_size = 10.0;
    double y_size = 10.0;
    double z_size = 10.0;
    Eigen::Vector3d origin(0.0, 0.0, 0.0);
    double inflateValue = 0.0;
    _sdf_map->initMap(x_size,y_size,z_size,resolution,origin,inflateValue);



    //添加未占据的元素的位置
    Eigen::Vector3d unoccupied_pos(2.0, 2.0, 2.0);
    _sdf_map->addLaserPoints(unoccupied_pos, 0);

    //添加占据的元素的位置
    Eigen::Vector3d obstacle_pos1(6, 6.0, 0.1);
    _sdf_map->addLaserPoints(obstacle_pos1, 1);

    //添加占据的元素的位置
    Eigen::Vector3d obstacle_pos2(3, 3.0, 0.1);
    _sdf_map->addLaserPoints(obstacle_pos2, 1);

    _sdf_map->startUpdateMapInfo();
    
    //  Eigen::Vector3d grad;
    // Eigen::Vector3d obstacle_pos_check(3.59521, 3.59829, 0);
    // double dis = _sdf_map->getDistWithGradTrilinear(obstacle_pos_check,  grad );

    // int status = _sdf_map->getInflateOccupancy(obstacle_pos_check);

    // std::cout << "dis " << dis << "status = "<< status << std::endl;

    //return 0;
    vector<Eigen::Vector3d> traj_pts;

   // for(int i = 0;i< 5;i++)
    float i = 0;
    float j = 0;
    std::vector<double> global_x;
    std::vector<double> global_y;

    while(j < 10)
    {

        Vector3d plan_pt(j, j, 0.1);
        traj_pts.push_back(plan_pt);
        j+= 0.1;
        global_x.push_back(j);
        global_y.push_back(j);
    }
    // while(j < 7)
    // {

    //     Vector3d plan_pt(j, 0, 0.2);
    //     traj_pts.push_back(plan_pt);
    //     j+= 0.1;
    //     global_x.push_back(j);
    //     global_y.push_back(0);
    // }
   
    // while(i < 10)
    // {

    //     Vector3d plan_pt(j, i, 0.2);
    //     traj_pts.push_back(plan_pt);
    //     i+= 0.1;
    //    // i++;
    //     global_x.push_back(j);
    //     global_y.push_back(i);
    // }

   
   #if 1

    MatrixXd acc(2, 3), vel(2, 3);

    Vector3d start_vel(0.0,     0,     0);
    Vector3d target_vel(0.0,    0,      0);

    Vector3d start_acc(0.0, 0.0, 0.0);
    Vector3d target_acc(0.0, 0.0,  0.0);

    vel.row(0) = start_vel;
    vel.row(1) = target_vel;

    acc.row(0) = start_acc;
    acc.row(1) = target_acc;

    double plan_time_interval  = 0.1;
    double check_time_interval = 0.1;

    //rcvReplanRequestCallBack();
    double local_to_global_time = 0;
    auto start = std::chrono::system_clock::now();
    gradLocalReplan(traj_pts, plan_time_interval, vel, acc, local_to_global_time);
    auto end = std::chrono::system_clock::now();
	auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	printf("MotionPlanner Total Running Time: %d  ms \n", elapsed.count());

    std::vector<double> ctrl_x;
    std::vector<double> ctrl_y;

    for(int i = 0; i < (int)ctrl_pts_Vis.rows(); i++)
    {   
                // std::cout << "pt ="  << ctrl_pts_Vis(i, 0) << " " << ctrl_pts_Vis(i, 1)  << std::endl;  
                ctrl_x.push_back(ctrl_pts_Vis(i, 0));
                ctrl_y.push_back(ctrl_pts_Vis(i, 1));
    }

   #endif
    //show
    std::map<std::string, std::string> keywords1;
    keywords1.insert(std::pair<std::string, std::string>("label", "globaltraj"));
    keywords1.insert(std::pair<std::string, std::string>("linewidth", "3.5"));
    plt::plot(global_x, global_y, keywords1);

    std::map<std::string, std::string> keywords2;
    keywords2.insert(std::pair<std::string, std::string>("label", "localtraj"));
    keywords2.insert(std::pair<std::string, std::string>("linewidth", "3.5"));

    plt::scatter(ctrl_x,ctrl_y);
    plt::plot(local_plan_x, local_plan_y, keywords2);
    
    vector<double> obs_x;
    vector<double> obs_y;
    vector<double> color;
    obs_x.push_back(obstacle_pos1[0]);
    obs_y.push_back(obstacle_pos1[1]);
    color.push_back(1.0);

    obs_x.push_back(obstacle_pos2[0]);
    obs_y.push_back(obstacle_pos2[1]);
    color.push_back(1.0);
    double point_size = 100.0;
    plt::scatter_colored(obs_x, obs_y,color,point_size,{{"cmap", "viridis"}});
    plt::legend();
    plt::show();


    // ros::spin();
    return 0;
}

#if 1
void drawBspline(NonUniformBspline bspline, int id,
                 Eigen::Vector4d color, bool show_ctrl_pts = false,
                 double size2 = 0.1, int id2 = 0,
                 Eigen::Vector4d color2 = Eigen::Vector4d(1, 1, 0, 1)) 
{
    vector<Eigen::Vector3d> traj_pts;
    double tm, tmp;
    bspline.getRegion(tm, tmp);

    NonUniformBspline vel_traj = bspline.getDerivative();  
    NonUniformBspline acc_traj = vel_traj.getDerivative();  
    
    for (double t = tm; t <= tmp; t += 0.01) 
    {
        Eigen::Vector3d pt = bspline.evaluateDeBoor(t);
        Vector3d  vel = vel_traj.evaluateDeBoor(t);
        //acc = acc_traj.evaluateDeBoor(t_bspline_start + t);
        traj_pts.push_back(pt);
        std::cout << "traj: " << pt << std::endl;
        std::cout << "vel =" << vel << std::endl;
        local_plan_x.push_back(pt[0]);
        local_plan_y.push_back(pt[1]);
    }

    // draw the control point
    if (!show_ctrl_pts) return;

    Eigen::MatrixXd ctrl_pts = bspline.getControlPoint();

    vector<Eigen::Vector3d> ctp;
    for (int i = 0; i < int(ctrl_pts.rows()); ++i) {
    Eigen::Vector3d pt = ctrl_pts.row(i).transpose();
    ctp.push_back(pt);
    }
    //displayPathWithColor(ctp, color2, id2);
}

#endif

#if 0
void visBsplineInitialization(const MatrixXd & ctrl_pts)
{
    visualization_msgs::Marker ctrl_pts_vis;

    ctrl_pts_vis.header.stamp       = ros::Time::now();
    ctrl_pts_vis.header.frame_id    = _frame_id;

    ctrl_pts_vis.id = 0;
    ctrl_pts_vis.ns = "local_replan/B-spline_initialization";
    ctrl_pts_vis.type = visualization_msgs::Marker::SPHERE_LIST;

    ctrl_pts_vis.action = visualization_msgs::Marker::ADD;
    ctrl_pts_vis.scale.x = _vis_traj_width / 2.0;
    ctrl_pts_vis.scale.y = _vis_traj_width / 2.0;
    ctrl_pts_vis.scale.z = _vis_traj_width / 2.0;
    ctrl_pts_vis.pose.orientation.x = 0.0;
    ctrl_pts_vis.pose.orientation.y = 0.0;
    ctrl_pts_vis.pose.orientation.z = 0.0;
    ctrl_pts_vis.pose.orientation.w = 1.0;

    ctrl_pts_vis.color.a = 1.0;
    ctrl_pts_vis.color.r = 1.0;
    ctrl_pts_vis.color.g = 0.0;
    ctrl_pts_vis.color.b = 0.0;
    
    ctrl_pts_vis.points.clear();

    geometry_msgs::Point pt;
    for(int i = 0; i < (int)ctrl_pts.rows(); i++)
    {   
        pt.x = ctrl_pts(i, 0);
        pt.y = ctrl_pts(i, 1);
        pt.z = ctrl_pts(i, 2);
        ctrl_pts_vis.points.push_back(pt);
    }

    //ROS_INFO("[local_replan] The length of the trajectory; %fm.", traj_len);
    _vis_initial_b_spline.publish(ctrl_pts_vis);
}
#endif

#if 0

void visFillMinima(Eigen::Vector3d center, Eigen::Vector3d sc) {
  static int idx = 0;
  idx++;

  visualization_msgs::Marker cube;
  cube.header.frame_id = _frame_id;
  cube.header.stamp = ros::Time::now();
  cube.type = visualization_msgs::Marker::CUBE;
  cube.action = visualization_msgs::Marker::ADD;
  cube.id = idx;

  cube.pose.orientation.w = 1.0;
  cube.color.b = 1.0, cube.color.a = 1.0, cube.color.r = 0.0,
  cube.color.g = 0.0;

  cube.pose.position.x = center(0);
  cube.pose.position.y = center(1);
  cube.pose.position.z = center(2);

  cube.scale.x = sc(0);
  cube.scale.y = sc(1);
  cube.scale.z = sc(2);

  _vis_fill_esdf_pub.publish(cube);
}

#endif
