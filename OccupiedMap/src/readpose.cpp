#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include <iomanip>

struct PoseData {
    double time;
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
};

std::vector<PoseData> loadTumData(const std::string& filename) {
    std::vector<PoseData> poses;
    std::ifstream file(filename);
    if (!file.is_open()) {
        ROS_ERROR("Failed to open TUM data file: %s", filename.c_str());
        return poses;
    }

    double time, tx, ty, tz, qx, qy, qz, qw;
    while (file >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw) {
        PoseData pose;
        pose.time = time;
        std::cout << std::fixed << std::setprecision(6) <<  time <<std::endl;
        pose.position = Eigen::Vector3d(tx, ty, tz);
        pose.orientation = Eigen::Quaterniond(qw, qx, qy, qz);
        poses.push_back(pose);
    }
    file.close();
    return poses;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tum_to_odometry");
    ros::NodeHandle nh("~");

    std::string tum_file_path;
    nh.param<std::string>("tum_file_path", tum_file_path, "/home/levy/pcl_ws/src/pcl_01/data/0605/data3_gt.txt");

    ros::Publisher odometry_pub = nh.advertise<nav_msgs::Odometry>("/Odometry", 1);

    std::vector<PoseData> tumData = loadTumData(tum_file_path);
    if (tumData.empty()) {
        ROS_ERROR("No TUM data loaded. Exiting.");
        return -1;
    }

    ros::Rate rate(10); // Adjust the rate as needed
    for (const auto& pose : tumData) {
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time(pose.time);
        odom.header.frame_id = "odom";

        odom.pose.pose.position.x = pose.position.x();
        odom.pose.pose.position.y = pose.position.y();
        odom.pose.pose.position.z = pose.position.z();
        odom.pose.pose.orientation.x = pose.orientation.x();
        odom.pose.pose.orientation.y = pose.orientation.y();
        odom.pose.pose.orientation.z = pose.orientation.z();
        odom.pose.pose.orientation.w = pose.orientation.w();

        odometry_pub.publish(odom);
        rate.sleep();
    }

    return 0;
}