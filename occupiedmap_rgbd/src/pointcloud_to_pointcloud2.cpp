#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class PointCloudConverter {
public:
    PointCloudConverter() {
        sub_ = nh_.subscribe("/pose_graph/orig_cloud", 1, &PointCloudConverter::callback, this);
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pose_graph/orig_cloud2", 1);
    }

    void callback(const sensor_msgs::PointCloud::ConstPtr& input) {
        // 首先将 sensor_msgs::PointCloud 转换为 sensor_msgs::PointCloud2
        sensor_msgs::PointCloud2 intermediate;
        sensor_msgs::convertPointCloudToPointCloud2(*input, intermediate);

        // 然后将 sensor_msgs::PointCloud2 转换为 pcl::PointCloud<pcl::PointXYZ>
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(intermediate, *cloud);

        // 将 pcl::PointCloud<pcl::PointXYZ> 转换回 sensor_msgs::PointCloud2 以发布
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*cloud, output);
        output.header = input->header;
        pub_.publish(output);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_converter");
    PointCloudConverter converter;
    ros::spin();
    return 0;
}
