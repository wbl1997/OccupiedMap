#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/common.h>

using namespace std;

class Point2pgm
{
public:
    Point2pgm();
    Point2pgm(ros::NodeHandle& nh);  // 带有NodeHandle参数的构造函数
    ~Point2pgm();

    void initializeOccupancyGrid();

    void PassThroughFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pcd_cloud,pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_after_PassThrough, const nav_msgs::Odometry& odom);
    void RadiusOutlierFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_after_PassThrough, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_after_Radius);
    void detectLines(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_line_opt);
    void OccupancyMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const nav_msgs::Odometry& odom);

    nav_msgs::OccupancyGrid map_msg;
    cv::Mat occupancy_grid;
    double thre_min;
    double thre_max;
    int flag_pass_through;
    std::string map_save_path;
    double map_resolution;
    double thre_radius;
    int min_neighbors;
    int bound_up;
    int bound_down;
    int bound_left;
    int bound_right;
    int Offset_x;
    int Offset_y;
    double angle_threshold = 0.05;

};