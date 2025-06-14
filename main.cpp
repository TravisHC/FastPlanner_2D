/*
 * @Function:using Trajectory Optimize Base Bspline to Generate Trajectory
 * @Create by:juchunyu@qq.com
 * @Date:2025-05-10 18:38:01
 */
#include <iostream>
#include <string>
#include <vector>
#include <cmath>

#include "PlannerInterface.h"
#include <opencv2/opencv.hpp>  // 添加OpenCV头文件

using namespace Planner;
using namespace cv;  // OpenCV命名空间

// 颜色定义
const Scalar COLOR_GLOBAL_TRAJ = Scalar(255, 0, 0);    // 蓝色 - 全局轨迹
const Scalar COLOR_LOCAL_TRAJ = Scalar(0, 0, 255);     // 红色 - 局部轨迹
const Scalar COLOR_OBSTACLE = Scalar(0, 255, 0);       // 绿色 - 障碍物

// 坐标转换函数：将世界坐标转换为图像坐标
Point2f worldToImage(const double x, const double y, 
                     const double resolution, const Point2f& origin) {
    return Point2f((x - origin.x) / resolution, 
                   (y - origin.y) / resolution);
}

int main()
{
    std::vector<double> global_x;
    std::vector<double> global_y;

    std::vector<double> local_plan_x;
    std::vector<double> local_plan_y;

    std::vector<double> obs_x;
    std::vector<double> obs_y;

    PlannerInterface plan;
    
    //初始化控制参数
    double max_vel = 1.0;
    double max_acc = 1.0;
    plan.initParam(max_vel,max_acc);

    //初始化ESDF地图
    double resolution = 0.1;
    double x_size = 10.0;
    double y_size = 10.0;
    double z_size = 10.0;
    Eigen::Vector3d origin(0.0, 0.0, 0.0);
    double inflateValue = 0.0;
    plan.initEsdfMap(x_size,y_size,z_size,resolution,origin,inflateValue);
  
    //添加障碍物点云
    std::vector<ObstacleInfo> obstacle;
    ObstacleInfo Obstemp;
    Obstemp.x = 6;
    Obstemp.y = 6;
    obstacle.push_back(Obstemp);
    Obstemp.x = 3.0;
    Obstemp.y = 3.0;
    obstacle.push_back(Obstemp);

    plan.setObstacles(obstacle);

    //添加全局路径点
    std::vector<PathPoint> global_plan_traj;
    
    global_x.clear();
    global_y.clear();

    float j = 0;

    while(j < 10)
    {
        PathPoint tempPoint;
        tempPoint.x = j;
        tempPoint.y = j;

        global_plan_traj.push_back(tempPoint);
  
        j += 0.1;
        global_x.push_back(j);
        global_y.push_back(j);
    }

    plan.setPathPoint(global_plan_traj);
    
    //开始规划
    plan.makePlan();
    
    //获取规划结果
    std::vector<PathPoint> plan_traj_res;
    plan.getLocalPlanTrajResults(plan_traj_res);
    
    local_plan_x.clear();
    local_plan_y.clear();

    for(int i = 0; i < plan_traj_res.size(); i++)
    {
        local_plan_x.push_back(plan_traj_res[i].x);
        local_plan_y.push_back(plan_traj_res[i].y);
    }

    // 显示参数
    const double DISPLAY_SCALE = 1.0;  // 显示缩放因子
    const int WINDOW_WIDTH = 1000 * 1e3;     // 窗口最小宽度
    const int WINDOW_HEIGHT = 800 * 1e3;     // 窗口最小高度

    // OpenCV可视化
    int img_width = static_cast<int>(x_size / resolution);
    int img_height = static_cast<int>(y_size / resolution);
    
    // 确保图像尺寸不超过窗口大小
    if (img_width * DISPLAY_SCALE > WINDOW_WIDTH) {
        img_width = static_cast<int>(WINDOW_WIDTH / DISPLAY_SCALE);
    }
    if (img_height * DISPLAY_SCALE > WINDOW_HEIGHT) {
        img_height = static_cast<int>(WINDOW_HEIGHT / DISPLAY_SCALE);
    }
    
    Mat visualization = Mat::zeros(img_height, img_width, CV_8UC3);
    visualization.setTo(Scalar(255, 255, 255));  // 白色背景

    // 绘制全局轨迹
    for (size_t i = 0; i < global_x.size() - 1; i++) {
        Point2f p1 = worldToImage(global_x[i], global_y[i], resolution, Point2f(0, 0));
        Point2f p2 = worldToImage(global_x[i+1], global_y[i+1], resolution, Point2f(0, 0));
        line(visualization, p1, p2, COLOR_GLOBAL_TRAJ, 2, LINE_AA);
    }

    // 绘制局部轨迹
    for (size_t i = 0; i < local_plan_x.size() - 1; i++) {
        Point2f p1 = worldToImage(local_plan_x[i], local_plan_y[i], resolution, Point2f(0, 0));
        Point2f p2 = worldToImage(local_plan_x[i+1], local_plan_y[i+1], resolution, Point2f(0, 0));
        line(visualization, p1, p2, COLOR_LOCAL_TRAJ, 2, LINE_AA);
    }

    // 绘制障碍物
    for (const auto& obs : obstacle) {
        Point2f center = worldToImage(obs.x, obs.y, resolution, Point2f(0, 0));
        circle(visualization, center, 5, COLOR_OBSTACLE, -1, LINE_AA);
    }

    // 添加图例
    putText(visualization, "Global Trajectory", Point(10, 20), 
            FONT_HERSHEY_SIMPLEX, 0.5, COLOR_GLOBAL_TRAJ, 1, LINE_AA);
    putText(visualization, "Local Trajectory", Point(10, 45), 
            FONT_HERSHEY_SIMPLEX, 0.5, COLOR_LOCAL_TRAJ, 1, LINE_AA);
    putText(visualization, "Obstacles", Point(10, 70), 
            FONT_HERSHEY_SIMPLEX, 0.5, COLOR_OBSTACLE, 1, LINE_AA);

    // 创建可调整大小的窗口
    namedWindow("Trajectory Visualization", WINDOW_NORMAL);
    
    // 设置窗口初始大小
    resizeWindow("Trajectory Visualization", 
                static_cast<int>(img_width * DISPLAY_SCALE), 
                static_cast<int>(img_height * DISPLAY_SCALE));
    
    // 显示结果
    imshow("Trajectory Visualization", visualization);
    
    // 保存原始大小的图像
    imwrite("trajectory_result.png", visualization);
    
    // 提示用户操作
    std::cout << "按任意键退出..." << std::endl;
    
    // 等待按键退出
    waitKey(0);

    return 0;
}