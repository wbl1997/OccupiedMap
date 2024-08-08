#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
#include <iomanip>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <thread>

#include "point2pgm.h"


ros::Publisher pub_pointmap;
ros::Publisher pub_marker;
std::string odom_topic, pointcloud_topic, state_topic;

queue<nav_msgs::Odometry::ConstPtr> odom_buffer;
queue<sensor_msgs::PointCloud2ConstPtr> cloud_buffer;
std::deque<std::string> state_buffer;
Point2pgm p2m;

const int WINDOW_SIZE = 2;              // 定义滑动窗口大小
const std::string STATE_UP = "up";      // 定义状态字符串
const std::string STATE_DOWN = "down";
const std::string STATE_FLAT = "flat";
int state_flag = 0;                     // 定义状态标志

int map_frame_count = 0; // 全局变量，用于记录已建图的帧数

bool Is_detectline = true;

// 计算状态变化
void checkStateChanges()
{
    int count_updown = 0;
    int count_flat = 0;

    for (const auto& state : state_buffer) {
        if (state == STATE_UP || state == STATE_DOWN) {
            count_updown++;
        } else if (state == STATE_FLAT) {
            count_flat++;
        }
    }

    // 检查条件并输出提示
    if (count_updown >= 2 && state_flag == 0) {
        ROS_INFO("Detected continuous 'up down' states.");
        state_flag = 1;
    } else if(count_flat >= 2 && state_flag == 1) {
        ROS_INFO("Detected continuous 'flat'.");
        p2m.initializeOccupancyGrid();
        state_flag = 0;
    }
}

void PcdToWorld(const pcl::PointCloud<pcl::PointXYZI>::Ptr pcd_cloud, const nav_msgs::Odometry odometry, pcl::PointCloud<pcl::PointXYZI>::Ptr& world_cloud)
{
    Eigen::Matrix3d Rs = Eigen::Quaterniond(
        odometry.pose.pose.orientation.w,
        odometry.pose.pose.orientation.x,
        odometry.pose.pose.orientation.y,
        odometry.pose.pose.orientation.z
    ).toRotationMatrix();
    
    Eigen::Vector3d Ps;
    Ps << odometry.pose.pose.position.x,
          odometry.pose.pose.position.y,
          odometry.pose.pose.position.z;

    Eigen::Matrix4f RPs = Eigen::Matrix4f::Identity();
    RPs.block<3, 3>(0, 0) = Rs.cast<float>(); // 将旋转矩阵转换为 float 类型
    RPs.block<3, 1>(0, 3) = Ps.cast<float>(); // 将平移向量转换为 float 类型

    // 将点云从 IMU 坐标系转换到世界坐标系下
    pcl::transformPointCloud(*pcd_cloud, *world_cloud, RPs);
}

void marker_publish(const nav_msgs::Odometry odometry)
{
    Eigen::Quaterniond R = Eigen::Quaterniond(odometry.pose.pose.orientation.w, odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";  // Use the same frame as the TF transform
    marker.header.stamp = odometry.header.stamp;
    marker.ns = "robot";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = odometry.pose.pose;
    Eigen::Quaterniond q(0, 0.7071, 0, 0.7071); // Quaternion representing a 90 degree rotation around the x-axis
    Eigen::Quaterniond rotated_q = R * q; // Rotate the marker's orientation by multiplying with the current orientation
    marker.pose.orientation.x = rotated_q.x();
    marker.pose.orientation.y = rotated_q.y();
    marker.pose.orientation.z = rotated_q.z();
    marker.pose.orientation.w = rotated_q.w();

    // 定义一个四元数表示旋转，使箭头指向 Y 轴
    marker.scale.x = 1.1;  // Length of the arrow
    marker.scale.y = 0.1;  // Width of the arrow
    marker.scale.z = 0.1;  // Height of the arrow
    marker.color.a = 1.0;  // Alpha
    marker.color.r = 1.0;  // Red
    marker.color.g = 0.0;  // Green
    marker.color.b = 0.0;  // Blue
    pub_marker.publish(marker);
}

std::pair<nav_msgs::Odometry, sensor_msgs::PointCloud2ConstPtr>
getMeasurements()
{
    std::pair<nav_msgs::Odometry, sensor_msgs::PointCloud2ConstPtr> measurement;

    while (true)
    {
        if (odom_buffer.empty() || cloud_buffer.empty()) 
        {   
            //ROS_WARN("empty buffer");
            return measurement; 
        }
        if (odom_buffer.front()->header.stamp.toSec() < cloud_buffer.front()->header.stamp.toSec() - 0.02)
        {
            std::cout << std::fixed << std::setprecision(5) << odom_buffer.front()->header.stamp.toSec() << std::endl<<std::endl;
            std::cout << std::fixed << std::setprecision(5) << cloud_buffer.front()->header.stamp.toSec() << std::endl;
            ROS_WARN("wait for cloud_buffer");
            odom_buffer.pop();
            continue;
        }
        else if (odom_buffer.front()->header.stamp.toSec() > cloud_buffer.front()->header.stamp.toSec() + 0.02)
        {
            std::cout << std::fixed << std::setprecision(5) << odom_buffer.front()->header.stamp.toSec() << std::endl;
            std::cout << std::fixed << std::setprecision(5) << cloud_buffer.front()->header.stamp.toSec() << std::endl;
            ROS_WARN("wait for odom_buffer");
            cloud_buffer.pop();
            continue;
        }
        else
        {
            // printf("odom.time: %f\n", odom_buffer.front()->header.stamp.toSec());
            // printf("cloud.time: %f\n", cloud_buffer.front()->header.stamp.toSec());
            nav_msgs::Odometry::ConstPtr odom_msg = odom_buffer.front();  
            odom_buffer.pop();
            sensor_msgs::PointCloud2ConstPtr cloud_msg = cloud_buffer.front();  
            cloud_buffer.pop();
            // measurement = std::make_pair(*odom_msg, cloud_msg);
            measurement = std::pair<nav_msgs::Odometry, sensor_msgs::PointCloud2ConstPtr>(*odom_msg, cloud_msg);
            break;
        }
    }
    return measurement;
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{   
    if (!odom_msg){
        ROS_WARN("Received null pointer in odom_callback. Ignoring...");
        return;
    }
    // m_buf.lock();
    odom_buffer.push(odom_msg);
    // m_buf.unlock();
}

void pcd_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) 
{
    if (!cloud_msg){
        ROS_WARN("Received null pointer in pcd_callback. Ignoring...");
        return;
    }
    // m_buf.lock();
    cloud_buffer.push(cloud_msg);
    // m_buf.unlock();
}

void state_callback(const std_msgs::String::ConstPtr& state_msg) 
{
    if (!state_msg){
        ROS_WARN("Received null in state_callback. Ignoring...");
        return;
    }

    state_buffer.push_back(state_msg->data);        // 添加新状态到滑动窗口

    if (state_buffer.size() > WINDOW_SIZE)          // 如果滑动窗口超过指定大小，移除最旧的状态
        state_buffer.pop_front();

    checkStateChanges();    // 检查状态变化
}

void process()
{
    // auto viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("viewer"));
    // viewer->setBackgroundColor(0, 0, 0);

    ros::Rate rate(10);
    while(true){
        std::pair<nav_msgs::Odometry, sensor_msgs::PointCloud2ConstPtr> measurement;
        measurement = getMeasurements();
        if (measurement.first.header.stamp.toSec() == 0.0 && !measurement.second)
        {
            rate.sleep();
            continue; 
        }
        nav_msgs::Odometry odometry = measurement.first;
        sensor_msgs::PointCloud2ConstPtr cloud_msg = measurement.second;

        pcl::PointCloud<pcl::PointXYZI>::Ptr pcd_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr world_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_after_PassThrough(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_after_Radius(new pcl::PointCloud<pcl::PointXYZI>);

        pcl::fromROSMsg(*cloud_msg, *pcd_cloud);

        marker_publish(odometry);
        PcdToWorld(pcd_cloud, odometry, world_cloud);

        // 高度重置楼层
        // float layer_height = odometry.pose.pose.position.z;
        // if (std::abs(layer_height - layer_height0) > 0.15)
        // {
        //     p2m.initializeOccupancyGrid();
        //     layer_height0 = layer_height;  
        // }

        // voxel filter 0.2m
        pcl::VoxelGrid<pcl::PointXYZI> voxel;
        voxel.setInputCloud(world_cloud);
        voxel.setLeafSize(0.1, 0.1, 0.1);
        voxel.filter(*world_cloud);

        p2m.PassThroughFilter(world_cloud, cloud_after_PassThrough, odometry);
        p2m.RadiusOutlierFilter(cloud_after_PassThrough, cloud_after_Radius);
        if(Is_detectline){
            // 直线检测+直线点优化（将直线点拉到对应直线上）+外点滤除
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_line_opt(new pcl::PointCloud<pcl::PointXYZI>);
            p2m.detectLines(cloud_after_Radius, cloud_line_opt);
            p2m.OccupancyMap(cloud_line_opt, odometry);
        }
        else{
            p2m.OccupancyMap(cloud_after_Radius, odometry);
        }
        pub_pointmap.publish(p2m.map_msg);

                // 增加帧计数并输出日志
        map_frame_count++;
        ROS_INFO("--------------------Map frame count---------------: %d", map_frame_count);


        // if(Is_detectline){
        //     // use viewer to show world_cloud
        //     viewer->addPointCloud<pcl::PointXYZI>(cloud_after_Radius, "world_cloud");
        //     viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "world_cloud");
        // }
        
        // viewer->spinOnce(100);
        // viewer->removeAllPointClouds();
        // viewer->removeAllShapes();
        rate.sleep();
    }
}

void pcd_callback0(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) 
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcd_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr world_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_after_PassThrough(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_after_Radius(new pcl::PointCloud<pcl::PointXYZI>);

    if (cloud_buffer.empty())
    {
        ROS_WARN("cloud_buffer is empty");
        return;
    }

    pcl::fromROSMsg(*cloud_msg, *pcd_cloud);

    if (odom_buffer.empty())
    {
        ROS_WARN("odom_buffer is empty");
        return;
    }
    nav_msgs::Odometry odometry = *odom_buffer.front();
    odom_buffer.pop();

    marker_publish(odometry);

    PcdToWorld(pcd_cloud, odometry, world_cloud);
    p2m.PassThroughFilter(world_cloud, cloud_after_PassThrough, odometry);
    p2m.RadiusOutlierFilter(cloud_after_PassThrough, cloud_after_Radius);
    p2m.OccupancyMap(cloud_after_Radius, odometry);
    pub_pointmap.publish(p2m.map_msg);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "pcd2map_node");
    ros::NodeHandle nh("~"); // Use private node handle for parameters

    // Get parameters from the parameter server
    nh.param("odom_topic", odom_topic, std::string("/vins_estimator/keyframe_pose"));
    nh.param("pointcloud_topic", pointcloud_topic, std::string("/vins_estimator/depth_cloud"));
    nh.param("state_topic", state_topic, std::string("/state"));
    nh.param("is_detectline", Is_detectline, true);
    p2m = Point2pgm(nh);

    pub_pointmap = nh.advertise<nav_msgs::OccupancyGrid>("/map",1000);
    pub_marker = nh.advertise<visualization_msgs::Marker>("/marker", 1000);
    ros::Subscriber odom_sub = nh.subscribe(odom_topic, 1000, odom_callback);
    ros::Subscriber pointcloud_sub = nh.subscribe(pointcloud_topic, 1000, pcd_callback);
    ros::Subscriber state_sub = nh.subscribe(state_topic, 1000, state_callback);

    std::thread measurement_process(process);    
    ros::spin();

    return 0;
}
