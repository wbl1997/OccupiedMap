#include "point2pgm.h"

using namespace std;

Point2pgm::Point2pgm() {
    thre_max = 1.5;       // z轴高度滤波
    thre_min = -0.5;
    flag_pass_through = 0;  // 是否启用直通滤波
    map_resolution = 0.1;   // 地图分辨率
    thre_radius = 0.1;      // 半径滤波
    min_neighbors = 3;      // 最小邻近数

    bound_up = 10;  // map bound
    bound_down = -10;
    bound_left = 10;
    bound_right = -10;
    Offset_x = 10;
    Offset_y = 10;

    initializeOccupancyGrid();      // 初始化地图消息
}

Point2pgm::Point2pgm(ros::NodeHandle& nh) {
  
    // 从ROS参数服务器中加载参数
    nh.param("thre_max", thre_max, 1.5);
    nh.param("thre_min", thre_min, -0.5);
    nh.param("flag_pass_through", flag_pass_through, 0);
    nh.param("map_resolution", map_resolution, 0.1);
    nh.param("thre_radius", thre_radius, 0.1);
    nh.param("min_neighbors", min_neighbors, 3);

    nh.param("bound_up", bound_up, 10);
    nh.param("bound_down", bound_down, -10);
    nh.param("bound_left", bound_left, 10);
    nh.param("bound_right", bound_right, -10);
    nh.param("Offset_x", Offset_x, 10);
    nh.param("Offset_y", Offset_y, 10);

    initializeOccupancyGrid();      // 初始化地图消息
}

void Point2pgm::initializeOccupancyGrid() {
    map_msg.header.seq = 0;
    map_msg.info.resolution = map_resolution;
    map_msg.info.origin.position.x = bound_down; // x_min
    map_msg.info.origin.position.y = bound_right; // y_min
    map_msg.info.origin.position.z = 0.0;
    map_msg.info.origin.orientation.x = 0.0;
    map_msg.info.origin.orientation.y = 0.0;
    map_msg.info.origin.orientation.z = 0.0;
    map_msg.info.origin.orientation.w = 1.0;
    map_msg.info.width = int((bound_up - bound_down) / map_resolution);
    map_msg.info.height = int((bound_left - bound_right) / map_resolution);
    map_msg.data.resize(map_msg.info.width * map_msg.info.height, -1);

    occupancy_grid.create(map_msg.info.height, map_msg.info.width, CV_8S);
    occupancy_grid.setTo(cv::Scalar(-1));
}

Point2pgm::~Point2pgm(){};



void Point2pgm::PassThroughFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcd_cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_after_PassThrough, const nav_msgs::Odometry& odom)
{
    if(pcd_cloud->points.empty())
    {
      ROS_WARN("PassThrough points is empty!\n");
      return;
    }

    double thre_min_corrected = thre_min + odom.pose.pose.position.z;
    double thre_max_corrected = thre_max + odom.pose.pose.position.z;

    pcl::PassThrough<pcl::PointXYZ> passthrough;        //直通滤波器对点云进行处理

    passthrough.setInputCloud(pcd_cloud);               //输入点云
    passthrough.setFilterFieldName("z");                //对z轴进行操作
    passthrough.setFilterLimits(thre_min_corrected, thre_max_corrected);   //设置直通滤波器操作范围
    passthrough.setFilterLimitsNegative(flag_pass_through);       //true表示保留范围外，false表示保留范围内
    passthrough.filter(*cloud_after_PassThrough);       //执行滤波，过滤结果保存在 cloud_after_PassThrough
    // std::cout << "直通滤波后点云数据点数：" << cloud_after_PassThrough->points.size() << std::endl;
}

void Point2pgm::RadiusOutlierFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_after_PassThrough,pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_after_Radius)
{
    if(cloud_after_PassThrough->points.empty())
    {
      ROS_WARN("RadiusOutlier points is empty!\n");
      return;
    }

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusoutlier;  //创建滤波器

    radiusoutlier.setInputCloud(cloud_after_PassThrough);    //设置输入点云
    radiusoutlier.setRadiusSearch(thre_radius);                   //设置radius为100的范围内找临近点
    radiusoutlier.setMinNeighborsInRadius(min_neighbors);       //设置查询点的邻域点集数小于2的删除
    radiusoutlier.filter(*cloud_after_Radius);
    // std::cout << "半径滤波后点云数据点数：" << cloud_after_Radius->points.size() << std::endl;
}

void Point2pgm::OccupancyMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const  nav_msgs::Odometry& odom)
{
  map_msg.header.stamp = ros::Time::now();
  map_msg.header.frame_id = "world";
  map_msg.info.map_load_time = ros::Time::now();

  if(cloud->points.empty())
  {
    ROS_WARN("OccupancyMap points is empty!\n");
    return;
  }

  double x_min, x_max, y_min, y_max;
  for(int i = 0; i < cloud->points.size() - 1; i++)
  {
    if(i == 0)
    {
      x_min = x_max = cloud->points[i].x;
      y_min = y_max = cloud->points[i].y;
    }

    double x = cloud->points[i].x;
    double y = cloud->points[i].y;

    if (x - odom.pose.pose.position.x > 20 || x - odom.pose.pose.position.x < -20 || y - odom.pose.pose.position.y > 20 || y - odom.pose.pose.position.y < -20)
    {
      continue;
    }

    if(x < x_min) x_min = x;
    if(x > x_max) x_max = x;
    if(y < y_min) y_min = y;
    if(y > y_max) y_max = y;
  }

  // map_msg.info.width = int((x_max - x_min) / map_resolution);
  // map_msg.info.height = int((y_max - y_min) / map_resolution);

  if(x_max > bound_up) {
    bound_up += Offset_x;
    std::cout << "bound_up:" << bound_up << std::endl;

    map_msg.info.width = int((bound_up - bound_down) / map_resolution);
    // map_msg.info.origin.position.y = bound_right; //y_min;
    map_msg.data.assign(map_msg.info.width * map_msg.info.height, -1);


    cv::Mat new_occupancy_grid(map_msg.info.height, map_msg.info.width, CV_8S, cv::Scalar(-1));    // 创建新的 occupancy_grid
    for (int i = 0; i < occupancy_grid.cols; i++) {    // 复制现有的数据到新的 occupancy_grid 中
        for (int j = 0; j < occupancy_grid.rows; j++) {
            map_msg.data[i + j * map_msg.info.width] = occupancy_grid.at<int8_t>(j, i);
            new_occupancy_grid.at<int8_t>(j, i) = occupancy_grid.at<int8_t>(j, i);
        }
    }
    occupancy_grid = new_occupancy_grid.clone();    // 更新 occupancy_grid
  }
  if(y_max > bound_left) {
    bound_left += Offset_y;
    std::cout << "bound_left:" << bound_left << std::endl;

    int Offset = Offset_y / map_resolution;
    map_msg.info.height = int((bound_left - bound_right) / map_resolution);
    map_msg.data.resize(map_msg.info.width * map_msg.info.height, -1);

    // occupancy_grid.resize(map_msg.info.width, std::vector<int>(map_msg.info.height, -1));
    cv::Mat new_occupancy_grid(map_msg.info.height, map_msg.info.width, CV_8S, cv::Scalar(-1));
    for (int x = 0; x < occupancy_grid.cols; x++) {    // 复制现有的数据到新的 occupancy_grid 中
        for (int y = 0; y < occupancy_grid.rows; y++) {
            new_occupancy_grid.at<int8_t>(y, x) = occupancy_grid.at<int8_t>(y, x);
        }
    }
    occupancy_grid = new_occupancy_grid.clone();    // 更新 occupancy_grid


  }
  if(x_min < bound_down) {
    bound_down -= Offset_x;
    std::cout << "bound_down:" << bound_down << std::endl;
  
    int Offset = Offset_x / map_resolution;
    map_msg.info.width = int((bound_up - bound_down) / map_resolution);
    map_msg.info.origin.position.x = bound_down; //x_min;
    map_msg.data.assign(map_msg.info.width * map_msg.info.height, -1);

    cv::Mat new_occupancy_grid(map_msg.info.height, map_msg.info.width, CV_8S, cv::Scalar(-1)); // 创建一个与地图大小相同的新 Mat 对象，初始值为 -1
    for (int x = 0; x < occupancy_grid.cols ; x++) {     // 修正此处遍历条件
        for (int y = 0; y < occupancy_grid.rows; y++) {
            int new_x = x + Offset;     // 计算新的 x 坐标
            if (occupancy_grid.at<int8_t>(y, x) != -1) {     // 如果当前位置的值不为 -1
                if (new_x >= 0 && new_x < map_msg.info.width){          // 如果新的 x 坐标在地图范围内
                    map_msg.data[new_x + y * map_msg.info.width] = occupancy_grid.at<int8_t>(y, x);    // 将当前位置的值赋给新的 x 坐标对应的位置
                    new_occupancy_grid.at<int8_t>(y, new_x) = occupancy_grid.at<int8_t>(y, x);    // 更新 occupancy_grid
                }
            }
        }
    }
    occupancy_grid = new_occupancy_grid.clone(); // 将新的 occupancy_grid 赋值给 occupancy_grid
  }

  if(y_min < bound_right) {
    bound_right -= Offset_y;
    std::cout << "bound_right:" << bound_right << std::endl;

    int Offset = Offset_y / map_resolution;
    map_msg.info.height = int((bound_left - bound_right) / map_resolution);
    map_msg.info.origin.position.y = bound_right; //y_min;
    map_msg.data.assign(map_msg.info.width * map_msg.info.height, -1);


    cv::Mat new_occupancy_grid(map_msg.info.height, map_msg.info.width, CV_8S, cv::Scalar(-1));        // 创建新的 occupancy_grid
    for (int x = 0; x < occupancy_grid.cols; x++) {    // 复制现有的数据到新的 occupancy_grid 中
        for (int y = 0; y < occupancy_grid.rows; y++) {
            if (x >= 0 && x < map_msg.info.width && y >= 0 && y < map_msg.info.height - Offset && occupancy_grid.at<int8_t>(y, x) != -1) {
                int new_y = y + Offset;
                map_msg.data[x + new_y * map_msg.info.width] = occupancy_grid.at<int8_t>(y, x);
                new_occupancy_grid.at<int8_t>(new_y, x) = occupancy_grid.at<int8_t>(y, x);
            }
        }
    }
    occupancy_grid = new_occupancy_grid.clone();    // 更新 occupancy_grid
  }

  double x1 = odom.pose.pose.position.x;
  double y1 = odom.pose.pose.position.y;
  for(int iter = 0; iter < cloud->points.size(); iter++) {
    // 计算当前点云点到里程计位置的线段的离散化
    double x2 = cloud->points[iter].x;
    double y2 = cloud->points[iter].y;
    double dx = x2 - x1;
    double dy = y2 - y1;
    double distance = std::sqrt(dx * dx + dy * dy);

    // 将线段离散化为栅格
    int steps = static_cast<int>(std::ceil(distance / map_resolution));
    std::vector<int> indices_x(steps + 1);
    std::vector<int> indices_y(steps + 1);
    for (int step = 0; step <= steps; ++step) {
      double ratio = static_cast<double>(step) / steps;
      double x = x1 + ratio * dx;
      double y = y1 + ratio * dy;
      indices_x[step] = static_cast<int>((x - bound_down) / map_resolution);
      indices_y[step] = static_cast<int>((y - bound_right) / map_resolution);
    }

    // 将线段上的栅格值设置为0，点云所在位置的栅格值设置为100
    for (int step = 0; step <= steps; ++step) {
      int x = indices_x[step];
      int y = indices_y[step];
      if (x >= 0 && x < map_msg.info.width && y >= 0 && y < map_msg.info.height) {
        int value = (step < steps) ? 0 : 100;
        map_msg.data[y * map_msg.info.width + x] = value;
        occupancy_grid.at<int8_t>(y, x) = value;
      }
    }
  }
}

// void Point2pgm::OccupancyMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const nav_msgs::Odometry& odom) {
//     map_msg.header.stamp = ros::Time::now();
//     map_msg.header.frame_id = "world";
//     map_msg.info.map_load_time = ros::Time::now();
//     if (cloud->points.empty()) {
//         ROS_WARN("pcd is empty!\n");
//         return;
//     }
//     double x_min, x_max, y_min, y_max;
//     // 计算点云的边界
//     calculateCloudBoundary(cloud, x_min, y_min, x_max, y_max);
//     // 更新地图大小
//     updateMapSize(x_min, y_min, x_max, y_max);
//     // 根据点云和里程计更新占据地图
//     updateOccupancyGrid(cloud, odom, x_min, y_min);
// }

// void Point2pgm::calculateCloudBoundary(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double& x_min, double& y_min, double& x_max, double& y_max) {
//     x_min = x_max = cloud->points[0].x;
//     y_min = y_max = cloud->points[0].y;
//     for (size_t i = 1; i < cloud->points.size(); ++i) {
//         double x = cloud->points[i].x;
//         double y = cloud->points[i].y;
//         if (x < x_min) x_min = x;
//         if (x > x_max) x_max = x;
//         if (y < y_min) y_min = y;
//         if (y > y_max) y_max = y;
//     }
// }

// void Point2pgm::updateMapSize(double x_min, double y_min, double x_max, double y_max) {
//     // 计算地图的原始中心位置
//     double original_center_x = (bound_up + bound_down) / 2.0;
//     double original_center_y = (bound_left + bound_right) / 2.0;
//     // 扩展地图宽度和高度时计算需要增加的行列数
//     int rows_to_add = 0, cols_to_add = 0;
//     if (x_max > bound_up) {
//         bound_up += Offset_x;
//         int new_width = int((bound_up - bound_down) / map_resolution);
//         rows_to_add = new_width - map_msg.info.width;
//         map_msg.info.width = new_width;
//     }
//     if (y_max > bound_left) {
//         bound_left += Offset_y;
//         int new_height = int((bound_left - bound_right) / map_resolution);
//         cols_to_add = new_height - map_msg.info.height;
//         map_msg.info.height = new_height;
//     }
//     if (x_min < bound_down) {
//         bound_down -= Offset_x;
//         int new_width = int((bound_up - bound_down) / map_resolution);
//         rows_to_add = new_width - map_msg.info.width;
//         map_msg.info.width = new_width;
//     }
//     if (y_min < bound_right) {
//         bound_right -= Offset_y;
//         int new_height = int((bound_left - bound_right) / map_resolution);
//         cols_to_add = new_height - map_msg.info.height;
//         map_msg.info.height = new_height;
//     }
//     // 根据需要增加的行列数来更新地图数据和 occupancy_grid
//     if (rows_to_add != 0 || cols_to_add != 0) {
//         // 创建一个新的地图数据和 occupancy_grid
//         std::vector<int8_t> new_data(map_msg.info.width * map_msg.info.height, -1);
//         cv::Mat new_occupancy_grid(map_msg.info.height, map_msg.info.width, CV_8S, cv::Scalar(-1));
//         // 复制现有的数据到新的地图数据和 occupancy_grid 中
//         for (int y = 0; y < occupancy_grid.rows; ++y) {
//             for (int x = 0; x < occupancy_grid.cols; ++x) {
//                 int new_x = x, new_y = y;
//                 if (rows_to_add > 0 && x < map_msg.info.width - rows_to_add) {
//                     new_x += rows_to_add; // 向右移动
//                 }
//                 if (cols_to_add > 0 && y < map_msg.info.height - cols_to_add) {
//                     new_y += cols_to_add; // 向下移动
//                 }
//                 if (new_x >= 0 && new_x < map_msg.info.width && new_y >= 0 && new_y < map_msg.info.height) {
//                     new_data[new_x + new_y * map_msg.info.width] = map_msg.data[x + y * map_msg.info.width];
//                     new_occupancy_grid.at<int8_t>(new_y, new_x) = occupancy_grid.at<int8_t>(y, x);
//                 }
//             }
//         }
//         // 更新地图消息和 occupancy_grid
//         map_msg.data = std::move(new_data);
//         occupancy_grid = new_occupancy_grid.clone();
//         // 保持原始中心位置不变，调整原点位置
//         map_msg.info.origin.position.x = original_center_x - map_msg.info.width * map_resolution / 2.0;
//         map_msg.info.origin.position.y = original_center_y - map_msg.info.height * map_resolution / 2.0;
//     }
// }

// void Point2pgm::updateOccupancyGrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const nav_msgs::Odometry& odom, double x_min, double y_min) {
//     double x1 = odom.pose.pose.position.x;
//     double y1 = odom.pose.pose.position.y;
//     for (int iter = 0; iter < cloud->points.size(); iter++) {
//         // 计算当前点云点到里程计位置的线段的离散化
//         double x2 = cloud->points[iter].x;
//         double y2 = cloud->points[iter].y;
//         double dx = x2 - x1;
//         double dy = y2 - y1;
//         double distance = std::sqrt(dx * dx + dy * dy);
//         // 将线段离散化为栅格
//         int steps = static_cast<int>(std::ceil(distance / map_resolution));
//         std::vector<int> indices_x(steps + 1);
//         std::vector<int> indices_y(steps + 1);
//         for (int step = 0; step <= steps; ++step) {
//             double ratio = static_cast<double>(step) / steps;
//             double x = x1 + ratio * dx;
//             double y = y1 + ratio * dy;
//             indices_x[step] = static_cast<int>((x - bound_down) / map_resolution);
//             indices_y[step] = static_cast<int>((y - bound_right) / map_resolution);
//         }
//         // 将线段上的栅格值设置为0，点云所在位置的栅格值设置为100
//         for (int step = 0; step <= steps; ++step) {
//             int x = indices_x[step];
//             int y = indices_y[step];
//             if (x >= 0 && x < map_msg.info.width && y >= 0 && y < map_msg.info.height) {
//                 int value = (step < steps) ? 0 : 100;
//                 map_msg.data[y * map_msg.info.width + x] = value;
//                 occupancy_grid.at<int8_t>(y, x) = value;
//             }
//         }
//     }
// }