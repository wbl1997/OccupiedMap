#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <fstream>
#include <vector>
#include <deque>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <functional>

bool enable_logging, enable_sensor_0, enable_sensor_1, enable_sensor_2;
double passthrough_front_max_0, passthrough_front_min_0, passthrough_height_max_0, passthrough_height_min_0;
double passthrough_front_max_1, passthrough_front_min_1, passthrough_height_max_1, passthrough_height_min_1;
double passthrough_front_max_2, passthrough_front_min_2, passthrough_height_max_2, passthrough_height_min_2;
int outlier_mean_k_0, outlier_mean_k_1, outlier_mean_k_2;
double outlier_stddev_thresh_0, outlier_stddev_thresh_1, outlier_stddev_thresh_2;

Eigen::Matrix3d rotation_0, rotation_1, rotation_2;
Eigen::Vector3d translation_0, translation_1, translation_2;

int radar_frame_count = 0; // 全局变量，用于记录发布的/radar帧数

struct PoseData {
    double time;
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
};

struct PointCloudData {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
};

std::deque<PoseData> pose_data_queue;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    PoseData pose;
    pose.time = msg->header.stamp.toSec();
    pose.position = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    pose.orientation = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

    pose_data_queue.push_back(pose);
    if (pose_data_queue.size() > 1000) {  // Limit the queue size
        pose_data_queue.pop_front();
    }

    if (enable_logging) {
        ROS_INFO("Received odometry data at time %f", pose.time);
    }
}

Eigen::Matrix4d getTransformationMatrix(int sensor_id) {
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

    if (sensor_id == 0) {
        transform.block<3, 3>(0, 0) = rotation_0;
        transform.block<3, 1>(0, 3) = translation_0;
    } else if (sensor_id == 1) {
        transform.block<3, 3>(0, 0) = rotation_1;
        transform.block<3, 1>(0, 3) = translation_1;
    } else if (sensor_id == 2) {
        transform.block<3, 3>(0, 0) = rotation_2;
        transform.block<3, 1>(0, 3) = translation_2;
    }

    if (enable_logging) {
        ROS_INFO_STREAM("Transformation matrix for sensor " << sensor_id << ":\n" << transform);
    }
    return transform;
}

void applyPassthroughFilter(pcl::PointCloud<pcl::PointXYZI>& cloud, int sensor_id) {
    pcl::PassThrough<pcl::PointXYZI> pass_x, pass_z;
    pass_x.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>(cloud));
    pass_x.setFilterFieldName("x");
    if (sensor_id == 0) {
        pass_x.setFilterLimits(-passthrough_front_max_0, -passthrough_front_min_0);
    } else if (sensor_id == 1) {
        pass_x.setFilterLimits(-passthrough_front_max_1, -passthrough_front_min_1);
    } else if (sensor_id == 2) {
        pass_x.setFilterLimits(-passthrough_front_max_2, -passthrough_front_min_2);
    }
    pass_x.filter(cloud);

    pass_z.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>(cloud));
    pass_z.setFilterFieldName("z");
    if (sensor_id == 0) {
        pass_z.setFilterLimits(passthrough_height_min_0, passthrough_height_max_0);
    } else if (sensor_id == 1) {
        pass_z.setFilterLimits(passthrough_height_min_1, passthrough_height_max_1);
    } else if (sensor_id == 2) {
        pass_z.setFilterLimits(passthrough_height_min_2, passthrough_height_max_2);
    }
    pass_z.filter(cloud);

    if (enable_logging) {
        ROS_INFO("Applied passthrough filter for sensor %d", sensor_id);
    }
}

void applyOutlierRemovalFilter(pcl::PointCloud<pcl::PointXYZI>& cloud, int sensor_id) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>(cloud));
    if (sensor_id == 0) {
        sor.setMeanK(outlier_mean_k_0);
        sor.setStddevMulThresh(outlier_stddev_thresh_0);
    } else if (sensor_id == 1) {
        sor.setMeanK(outlier_mean_k_1);
        sor.setStddevMulThresh(outlier_stddev_thresh_1);
    } else if (sensor_id == 2) {
        sor.setMeanK(outlier_mean_k_2);
        sor.setStddevMulThresh(outlier_stddev_thresh_2);
    }
    sor.filter(cloud);
    if (enable_logging) {
        ROS_INFO("Applied outlier removal filter for sensor %d", sensor_id);
    }
}

void transformPointCloud(const pcl::PointCloud<pcl::PointXYZI>& input_cloud, pcl::PointCloud<pcl::PointXYZI>& output_cloud, const Eigen::Matrix4f& transform) {
    pcl::transformPointCloud(input_cloud, output_cloud, transform);
}

PoseData interpolatePose(double timestamp, const PoseData& pose1, const PoseData& pose2) {
    double ratio = (timestamp - pose1.time) / (pose2.time - pose1.time);
    PoseData interpolated_pose;
    interpolated_pose.time = timestamp;
    interpolated_pose.position = pose1.position + ratio * (pose2.position - pose1.position);
    interpolated_pose.orientation = pose1.orientation.slerp(ratio, pose2.orientation);
    if (enable_logging) {
        ROS_INFO("Interpolated pose at timestamp %f", timestamp);
    }
    return interpolated_pose;
}

void logPose(const std::string& label, const PoseData& pose) {
    if (enable_logging) {
        ROS_INFO("%s: time = %f, position = [%f, %f, %f], orientation = [%f, %f, %f, %f]",
                 label.c_str(), pose.time, pose.position.x(), pose.position.y(), pose.position.z(),
                 pose.orientation.x(), pose.orientation.y(), pose.orientation.z(), pose.orientation.w());
    }
}

std::deque<sensor_msgs::PointCloud2ConstPtr> pending_cloud_queue;

void radarCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                   ros::Publisher& transformed_cloud_pub, ros::Publisher& world_cloud_pub,
                   tf2_ros::TransformBroadcaster& broadcaster, nav_msgs::Path& path,
                   ros::Publisher& path_pub, ros::Publisher& odometry_pub,
                   std::deque<PointCloudData>& cloud_queue, int window_size, bool enable_passthrough, bool enable_outlier_removal,
                   int sensor_id, std::deque<PointCloudData>& combined_queue) {
    const int queue_max_size = 200;  // 队列大小设置为200
    pending_cloud_queue.push_back(cloud_msg);

    while (!pending_cloud_queue.empty()) {
        const auto& current_cloud_msg = pending_cloud_queue.front();
        double lidarTimestamp = current_cloud_msg->header.stamp.toSec();
        // ROS_INFO_STREAM("Processing point cloud with timestamp: " << std::fixed << std::setprecision(5) << lidarTimestamp);

        if (pose_data_queue.empty()) {
            ROS_WARN("No pose data available. Skipping this callback.");
            break;
        }

        // 输出pose_data_queue的开始和结束时间戳
        double startPoseTimestamp = pose_data_queue.front().time;
        double endPoseTimestamp = pose_data_queue.back().time;
        ROS_INFO_STREAM("NOW timestamp: " << std::fixed << std::setprecision(5) << lidarTimestamp);
        ROS_INFO_STREAM("Pose queue start timestamp: " << std::fixed << std::setprecision(5) << startPoseTimestamp);
        ROS_INFO_STREAM("Pose queue end timestamp: " << std::fixed << std::setprecision(5) << endPoseTimestamp);

        if (lidarTimestamp <= startPoseTimestamp) {
            ROS_WARN("Point cloud timestamp is earlier than pose queue start time. Removing this cloud.");
            pending_cloud_queue.pop_front();
            continue;
        }

        pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
        pcl::fromROSMsg(*current_cloud_msg, pcl_cloud);

        Eigen::Matrix4f rotation_matrix = getTransformationMatrix(sensor_id).cast<float>();
        pcl::PointCloud<pcl::PointXYZI> rotated_pcl_cloud;
        pcl::transformPointCloud(pcl_cloud, rotated_pcl_cloud, rotation_matrix);
        if (enable_passthrough) {
            applyPassthroughFilter(rotated_pcl_cloud, sensor_id);
            }   
        

        auto it = std::lower_bound(pose_data_queue.begin(), pose_data_queue.end(), lidarTimestamp,
                                   [](const PoseData& pose, double timestamp) {
                                       return pose.time < timestamp;
                                   });

        if (it == pose_data_queue.end() || it == pose_data_queue.begin()) {
            // 无法进行插值，跳过当前点云数据
            ROS_WARN("Cannot interpolate pose for timestamp. Skipping this cloud.");
            break;
        }

        const PoseData& pose2 = *it;
        const PoseData& pose1 = *(it - 1);
        PoseData interpolated_pose = interpolatePose(lidarTimestamp, pose1, pose2);

        logPose("Interpolated Pose", interpolated_pose);

        PointCloudData cloud_data;
        cloud_data.cloud = rotated_pcl_cloud;
        cloud_data.position = interpolated_pose.position;
        cloud_data.orientation = interpolated_pose.orientation;
        cloud_queue.push_back(cloud_data);
        combined_queue.push_back(cloud_data);

        if (window_size % 2 == 0) {
            window_size += 1;
        }

        int half_window_size = (window_size - 1) / 2;

        if (cloud_queue.size() > window_size) {
            cloud_queue.pop_front();
        }

        if (combined_queue.size() > window_size) {
            combined_queue.pop_front();
        }

        if (cloud_queue.size() < window_size || combined_queue.size() < window_size) {
            ROS_WARN("Cloud queue size is less than the window size. Skipping this callback.");
            break;
        }

        pcl::PointCloud<pcl::PointXYZI> merged_cloud;
        Eigen::Vector3d current_position = combined_queue[half_window_size].position;
        Eigen::Quaterniond current_orientation = combined_queue[half_window_size].orientation;
        Eigen::Matrix4f current_transform = (Eigen::Matrix4f) Eigen::Affine3d(current_orientation).matrix().cast<float>();
        current_transform.block<3,1>(0,3) = current_position.cast<float>();

        for (const auto& cloud_data : combined_queue) {
            Eigen::Affine3d relative_transform = Eigen::Affine3d(current_orientation.inverse() * cloud_data.orientation);
            relative_transform.translation() = current_orientation.inverse() * (cloud_data.position - current_position);
            pcl::PointCloud<pcl::PointXYZI> transformed_cloud;
            transformPointCloud(cloud_data.cloud, transformed_cloud, relative_transform.matrix().cast<float>());
            merged_cloud += transformed_cloud;
        }

        if (enable_outlier_removal) {
            applyOutlierRemovalFilter(merged_cloud, sensor_id);
        }

        sensor_msgs::PointCloud2 output_cloud;
        pcl::toROSMsg(merged_cloud, output_cloud);
        output_cloud.header.frame_id = "frame1";
        output_cloud.header.stamp = ros::Time::now();
        transformed_cloud_pub.publish(output_cloud);

        // 计数和日志记录
        radar_frame_count++;
        ROS_INFO("Published radar frame count: %d", radar_frame_count);

        
        Eigen::Affine3d world_transform = Eigen::Affine3d::Identity();
        world_transform.translate(current_position);
        world_transform.rotate(current_orientation);

        pcl::PointCloud<pcl::PointXYZI> world_cloud;
        pcl::transformPointCloud(merged_cloud, world_cloud, world_transform.cast<float>());

        sensor_msgs::PointCloud2 world_output_cloud;
        pcl::toROSMsg(world_cloud, world_output_cloud);
        world_output_cloud.header.frame_id = "world";
        world_output_cloud.header.stamp = ros::Time::now();
        world_cloud_pub.publish(world_output_cloud);

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = current_position.x();
        pose_stamped.pose.position.y = current_position.y();
        pose_stamped.pose.position.z = current_position.z();
        pose_stamped.pose.orientation.x = current_orientation.x();
        pose_stamped.pose.orientation.y = current_orientation.y();
        pose_stamped.pose.orientation.z = current_orientation.z();
        pose_stamped.pose.orientation.w = current_orientation.w();

        path.poses.push_back(pose_stamped);
        path_pub.publish(path);

        // Publish odometry message
        nav_msgs::Odometry odometry_msg;
        odometry_msg.header.stamp = ros::Time::now();
        odometry_msg.header.frame_id = "world";
        odometry_msg.pose.pose = pose_stamped.pose;
        odometry_msg.twist.twist.linear.x = 0;  // You may want to set the correct linear velocity
        odometry_msg.twist.twist.linear.y = 0;
        odometry_msg.twist.twist.linear.z = 0;
        odometry_msg.twist.twist.angular.x = 0;  // You may want to set the correct angular velocity
        odometry_msg.twist.twist.angular.y = 0;
        odometry_msg.twist.twist.angular.z = 0;

        odometry_pub.publish(odometry_msg);

        // 输出队列的大小
        ROS_INFO_STREAM("Pending cloud queue size: " << pending_cloud_queue.size());

        pending_cloud_queue.pop_front();  // 删除已经成功处理的点云数据

        // 确保队列大小不超过200
        if (pending_cloud_queue.size() > queue_max_size) {
            pending_cloud_queue.pop_front();
            ROS_WARN("Pending cloud queue exceeded maximum size. Dropping oldest entry.");
        }
    }
}



std::vector<double> parseVector(const std::string& str) {
    std::stringstream ss(str);
    std::string item;
    std::vector<double> result;
    while (std::getline(ss, item, ',')) {
        result.push_back(std::stod(item));
    }
    return result;
}

void loadParameters(ros::NodeHandle& nh) {
    nh.param("enable_logging", enable_logging, true);
    nh.param("passthrough_front_max_0", passthrough_front_max_0, 30.0);
    nh.param("passthrough_front_min_0", passthrough_front_min_0, 0.5);
    nh.param("passthrough_height_min_0", passthrough_height_min_0, -0.5);
    nh.param("passthrough_height_max_0", passthrough_height_max_0, 0.5);
    nh.param("passthrough_front_max_1", passthrough_front_max_1, 20.0);
    nh.param("passthrough_front_min_1", passthrough_front_min_1, 0.5);
    nh.param("passthrough_height_min_1", passthrough_height_min_1, -0.5);
    nh.param("passthrough_height_max_1", passthrough_height_max_1, 0.5);
    nh.param("passthrough_front_max_2", passthrough_front_max_2, 20.0);
    nh.param("passthrough_front_min_2", passthrough_front_min_2, 0.5);
    nh.param("passthrough_height_min_2", passthrough_height_min_2, -0.5);
    nh.param("passthrough_height_max_2", passthrough_height_max_2, 0.5);
    nh.param("outlier_mean_k_0", outlier_mean_k_0, 50);
    nh.param("outlier_stddev_thresh_0", outlier_stddev_thresh_0, 1.0);
    nh.param("outlier_mean_k_1", outlier_mean_k_1, 50);
    nh.param("outlier_stddev_thresh_1", outlier_stddev_thresh_1, 1.0);
    nh.param("outlier_mean_k_2", outlier_mean_k_2, 50);
    nh.param("outlier_stddev_thresh_2", outlier_stddev_thresh_2, 1.0);

    std::string rotation_str_0, rotation_str_1, rotation_str_2;
    std::string translation_str_0, translation_str_1, translation_str_2;

    nh.param("rotation_0", rotation_str_0, std::string("0,1,0,-1,0,0,0,0,1"));
    nh.param("translation_0", translation_str_0, std::string("0,0,0"));
    nh.param("rotation_1", rotation_str_1, std::string("1,0,0,0,1,0,0,0,1"));
    nh.param("translation_1", translation_str_1, std::string("0.02,0.13,-0.025"));
    nh.param("rotation_2", rotation_str_2, std::string("-1,0,0,0,1,0,0,0,-1"));
    nh.param("translation_2", translation_str_2, std::string("0.015,-0.06,-0.025"));

    rotation_0 = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(parseVector(rotation_str_0).data());
    translation_0 = Eigen::Map<Eigen::Vector3d>(parseVector(translation_str_0).data());
    rotation_1 = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(parseVector(rotation_str_1).data());
    translation_1 = Eigen::Map<Eigen::Vector3d>(parseVector(translation_str_1).data());
    rotation_2 = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(parseVector(rotation_str_2).data());
    translation_2 = Eigen::Map<Eigen::Vector3d>(parseVector(translation_str_2).data());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "radar_to_world");
    ros::NodeHandle nh("~");

    loadParameters(nh);

    bool enable_passthrough, enable_outlier_removal;
    int window_size;
    std::string radar_sub_topic_0, radar_pub_topic, world_pub_topic, radar_sub_topic_1, radar_sub_topic_2, path_pub_topic, odometry_topic;

    nh.param("enable_passthrough", enable_passthrough, true);
    nh.param("enable_outlier_removal", enable_outlier_removal, true);
    nh.param("window_size", window_size, 11);
    nh.param<std::string>("radar_sub_topic_0", radar_sub_topic_0, "/ti_mmwave/radar_scan_pcl_0");
    nh.param<std::string>("radar_pub_topic", radar_pub_topic, "/radar");
    nh.param<std::string>("world_pub_topic", world_pub_topic, "/world_radar");
    nh.param<std::string>("radar_sub_topic_1", radar_sub_topic_1, "/livox/lidar");
    nh.param<std::string>("radar_sub_topic_2", radar_sub_topic_2, "/ti_mmwave/radar_scan_pcl_AOP_1");
    nh.param<std::string>("odometry_topic", odometry_topic, "/Odometry");
    nh.param<std::string>("path_pub_topic", path_pub_topic, "/path");

    ros::Subscriber odometry_sub = nh.subscribe(odometry_topic, 1000, odomCallback);

    ros::Publisher transformed_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(radar_pub_topic, 1);
    ros::Publisher world_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(world_pub_topic, 1);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>(path_pub_topic, 1);
    ros::Publisher odometry_pub = nh.advertise<nav_msgs::Odometry>("/odometry", 1);  // Add odometry publisher

    tf2_ros::TransformBroadcaster broadcaster;
    std::deque<PointCloudData> cloud_queue_0, cloud_queue_1, cloud_queue_2, combined_queue;
    nav_msgs::Path path;
    path.header.frame_id = "world";

    ros::Subscriber radar_sub_0, radar_sub_1, radar_sub_2;

    nh.param("enable_sensor_0", enable_sensor_0, true);
    nh.param("enable_sensor_1", enable_sensor_1, true);
    nh.param("enable_sensor_2", enable_sensor_2, true);

    if (enable_sensor_0) {
        radar_sub_0 = nh.subscribe<sensor_msgs::PointCloud2>(radar_sub_topic_0, 1, 
            std::bind(&radarCallback, std::placeholders::_1, std::ref(transformed_cloud_pub), std::ref(world_cloud_pub),
            std::ref(broadcaster), std::ref(path), std::ref(path_pub), std::ref(odometry_pub), // Pass odometry publisher
            std::ref(cloud_queue_0), window_size, enable_passthrough, enable_outlier_removal, 0, std::ref(combined_queue)));
    }

    if (enable_sensor_1) {
        radar_sub_1 = nh.subscribe<sensor_msgs::PointCloud2>(radar_sub_topic_1, 1, 
            std::bind(&radarCallback, std::placeholders::_1, std::ref(transformed_cloud_pub), std::ref(world_cloud_pub),
            std::ref(broadcaster), std::ref(path), std::ref(path_pub), std::ref(odometry_pub), // Pass odometry publisher
            std::ref(cloud_queue_1), window_size, enable_passthrough, enable_outlier_removal, 1, std::ref(combined_queue)));
    }

    if (enable_sensor_2) {
        radar_sub_2 = nh.subscribe<sensor_msgs::PointCloud2>(radar_sub_topic_2, 1, 
            std::bind(&radarCallback, std::placeholders::_1, std::ref(transformed_cloud_pub), std::ref(world_cloud_pub),
            std::ref(broadcaster), std::ref(path), std::ref(path_pub), std::ref(odometry_pub), // Pass odometry publisher
            std::ref(cloud_queue_2), window_size, enable_passthrough, enable_outlier_removal, 2, std::ref(combined_queue)));
    }

    ros::spin();
    return 0;
}
