#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

std::vector<geometry_msgs::PoseStamped> pathData;
ros::Publisher cloud_pub;
ros::Publisher odom_pub;
tf::TransformBroadcaster* tf_broadcaster;

bool use_outlier_removal;
bool use_pass_through;
bool log_enabled;
int window_size;
double passthrough_z_min;
double passthrough_z_max;
double passthrough_x_min;
double passthrough_x_max;
int sor_mean_k;
double sor_stddev_thresh;

std::vector<pcl::PointCloud<pcl::PointXYZ>> cloud_buffer;
std::vector<geometry_msgs::Pose> pose_buffer;

void pathCallback(const nav_msgs::Path::ConstPtr& path_msg) {
    pathData = path_msg->poses;
    if (log_enabled) {
        ROS_INFO("Path data received with %lu poses", pathData.size());
    }
}

Eigen::Matrix4f poseToMatrix(const geometry_msgs::Pose& pose) {
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    Eigen::Quaternionf q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    transform.block<3, 3>(0, 0) = q.toRotationMatrix();
    transform(0, 3) = pose.position.x;
    transform(1, 3) = pose.position.y;
    transform(2, 3) = pose.position.z;
    return transform;
}

void processAndPublishCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud, const geometry_msgs::Pose& pose) {
    // Apply pass-through filters
    pcl::PointCloud<pcl::PointXYZ> filtered_cloud = cloud;
    if (use_pass_through) {
        pcl::PointCloud<pcl::PointXYZ> filtered_cloud_z;
        pcl::PassThrough<pcl::PointXYZ> pass_z;
        pass_z.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(cloud));
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(passthrough_x_min, passthrough_x_max);  // Limit Z axis
        pass_z.filter(filtered_cloud_z);

        pcl::PassThrough<pcl::PointXYZ> pass_y;
        pass_y.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(filtered_cloud_z));
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(passthrough_z_min, passthrough_z_max);  
        pass_y.filter(filtered_cloud);
    }

    // Apply statistical outlier removal
    if (use_outlier_removal) {
        pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(filtered_cloud));
        sor.setMeanK(sor_mean_k);  
        sor.setStddevMulThresh(sor_stddev_thresh); 
        sor.filter(cloud_filtered);
        filtered_cloud = cloud_filtered;
    }

    // Apply rotation matrix for 90 degrees X-axis rotation
    Eigen::Affine3f rotation = Eigen::Affine3f::Identity();
    rotation.rotate(Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitX())); // Rotate 90 degrees around the X-axis
    pcl::PointCloud<pcl::PointXYZ> rotated_cloud;
    pcl::transformPointCloud(filtered_cloud, rotated_cloud, rotation);

    // Convert the rotated PCL point cloud back to sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2 cloud_frame_adjusted;
    pcl::toROSMsg(rotated_cloud, cloud_frame_adjusted);
    cloud_frame_adjusted.header.stamp = ros::Time::now();
    cloud_frame_adjusted.header.frame_id = "frame1";

    cloud_pub.publish(cloud_frame_adjusted);

    // Create and publish Odometry message
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "frame1";
    odom_msg.pose.pose = pose;

    odom_pub.publish(odom_msg);

    // Publish tf transform
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
    transform.setRotation(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));
    tf_broadcaster->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "frame1"));
}

void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    double lidarTimestamp = cloud_msg->header.stamp.toSec();

    // Check for matching pose
    auto it = std::find_if(pathData.begin(), pathData.end(), [lidarTimestamp](const geometry_msgs::PoseStamped& p) {
        return std::abs(p.header.stamp.toSec() - lidarTimestamp) < 0.001;
    });

    if (it == pathData.end()) {
        if (log_enabled) {
            ROS_WARN("No corresponding pose data for the given lidar timestamp.");
        }
        return; // No valid corresponding pose data at this exact timestamp
    }

    const geometry_msgs::Pose& pose = it->pose;

    // Convert sensor_msgs::PointCloud2 to pcl::PointCloud
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*cloud_msg, pcl_cloud);

    // Add current cloud and pose to buffer
    cloud_buffer.push_back(pcl_cloud);
    pose_buffer.push_back(pose);

    // Adjust window size if even
    if (window_size % 2 == 0) {
        window_size += 1;
    }

    // If buffer is not yet full, process and publish the current cloud
    if (cloud_buffer.size() < window_size) {
        if (log_enabled) {
            ROS_INFO("Buffer not full yet. Current size: %lu, required: %d", cloud_buffer.size(), window_size);
        }
        processAndPublishCloud(pcl_cloud, pose);
        return;
    }

    // Determine the range of indices to combine
    int center_index = cloud_buffer.size() - 1;
    int half_window = (window_size - 1) / 2;
    int start_index = std::max(center_index - half_window, 0);
    int end_index = std::min(center_index + half_window, static_cast<int>(cloud_buffer.size()) - 1);

    // Combine clouds in the specified range with motion compensation
    pcl::PointCloud<pcl::PointXYZ> combined_cloud;
    Eigen::Matrix4f center_transform = poseToMatrix(pose_buffer[center_index]);

    for (int i = start_index; i <= end_index; ++i) {
        Eigen::Matrix4f transform = poseToMatrix(pose_buffer[i]);
        Eigen::Matrix4f relative_transform = center_transform.inverse() * transform;
        pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
        pcl::transformPointCloud(cloud_buffer[i], transformed_cloud, relative_transform);
        combined_cloud += transformed_cloud;
    }

    // Remove the oldest cloud and pose if buffer is full
    if (cloud_buffer.size() > window_size) {
        cloud_buffer.erase(cloud_buffer.begin());
        pose_buffer.erase(pose_buffer.begin());
    }

    // Process and publish the combined cloud
    processAndPublishCloud(combined_cloud, pose);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "depth_cloud_to_frame1_transformer");
    ros::NodeHandle nh("~"); // Use private node handle for parameters

    // Get parameters from the parameter server
    std::string path_topic, depth_cloud_topic, output_cloud_topic;
    nh.param("path_topic", path_topic, std::string("/pose_graph/pose_graph_path"));
    nh.param("depth_cloud_topic", depth_cloud_topic, std::string("/pose_graph/orig_cloud2"));
    nh.param("output_cloud_topic", output_cloud_topic, std::string("/rgbd"));
    nh.param("use_outlier_removal", use_outlier_removal, false);
    nh.param("use_pass_through", use_pass_through, false);
    nh.param("log_enabled", log_enabled, true);
    nh.param("window_size", window_size, 5); // Default window size is 5
    nh.param("passthrough_x_min", passthrough_x_min, 0.2);
    nh.param("passthrough_x_max", passthrough_x_max, 10.0);
    nh.param("passthrough_z_min", passthrough_z_min, -0.5);
    nh.param("passthrough_z_max", passthrough_z_max, 1.0);
    nh.param("sor_mean_k", sor_mean_k, 50);
    nh.param("sor_stddev_thresh", sor_stddev_thresh, 1.0);

    ros::Subscriber path_sub = nh.subscribe(path_topic, 1, pathCallback);
    ros::Subscriber depth_cloud_sub = nh.subscribe(depth_cloud_topic, 1, lidarCallback);
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(output_cloud_topic, 1);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 1);
    tf_broadcaster = new tf::TransformBroadcaster();

    if (log_enabled) {
        ROS_INFO("Node initialized with the following parameters:");
        ROS_INFO("Path topic: %s", path_topic.c_str());
        ROS_INFO("Depth cloud topic: %s", depth_cloud_topic.c_str());
        ROS_INFO("Output cloud topic: %s", output_cloud_topic.c_str());
        ROS_INFO("Use outlier removal: %s", use_outlier_removal ? "true" : "false");
        ROS_INFO("Use pass through: %s", use_pass_through ? "true" : "false");
        ROS_INFO("Log enabled: %s", log_enabled ? "true" : "false");
        ROS_INFO("Window size: %d", window_size);
        ROS_INFO("Passthrough X min: %f", passthrough_x_min);
        ROS_INFO("Passthrough X max: %f", passthrough_x_max);
        ROS_INFO("Passthrough Z min: %f", passthrough_z_min);
        ROS_INFO("Passthrough Z max: %f", passthrough_z_max);
        ROS_INFO("SOR mean k: %d", sor_mean_k);
        ROS_INFO("SOR stddev thresh: %f", sor_stddev_thresh);
    }

    ros::spin();
    delete tf_broadcaster;
    return 0;
}