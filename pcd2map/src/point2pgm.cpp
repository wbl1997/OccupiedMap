#include "point2pgm.h"
#include <pcl/features/normal_3d.h>
#include <thread>
#include <vector>

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
    //map_msg.data.resize(map_msg.info.width * map_msg.info.height, -1);

    map_msg.data.assign(map_msg.info.width * map_msg.info.height, -1);
    occupancy_grid.create(map_msg.info.height, map_msg.info.width, CV_8S);
    occupancy_grid.setTo(cv::Scalar(-1));
}

Point2pgm::~Point2pgm(){};



void Point2pgm::PassThroughFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pcd_cloud,pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_after_PassThrough, const nav_msgs::Odometry& odom)
{
    if(pcd_cloud->points.empty())
    {
      ROS_WARN("PassThrough points is empty!\n");
      return;
    }

    double thre_min_corrected = thre_min + odom.pose.pose.position.z;
    double thre_max_corrected = thre_max + odom.pose.pose.position.z;

    pcl::PassThrough<pcl::PointXYZI> passthrough;        //直通滤波器对点云进行处理

    passthrough.setInputCloud(pcd_cloud);               //输入点云
    passthrough.setFilterFieldName("z");                //对z轴进行操作
    passthrough.setFilterLimits(thre_min_corrected, thre_max_corrected);   //设置直通滤波器操作范围
    passthrough.setFilterLimitsNegative(flag_pass_through);       //true表示保留范围外，false表示保留范围内
    passthrough.filter(*cloud_after_PassThrough);       //执行滤波，过滤结果保存在 cloud_after_PassThrough
    // std::cout << "直通滤波后点云数据点数：" << cloud_after_PassThrough->points.size() << std::endl;
}

void Point2pgm::RadiusOutlierFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_after_PassThrough,pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_after_Radius)
{
    if(cloud_after_PassThrough->points.empty())
    {
      ROS_WARN("RadiusOutlier points is empty!\n");
      return;
    }

    pcl::RadiusOutlierRemoval<pcl::PointXYZI> radiusoutlier;  //创建滤波器

    radiusoutlier.setInputCloud(cloud_after_PassThrough);    //设置输入点云
    radiusoutlier.setRadiusSearch(thre_radius);                   //设置radius为100的范围内找临近点
    radiusoutlier.setMinNeighborsInRadius(min_neighbors);       //设置查询点的邻域点集数小于2的删除
    radiusoutlier.filter(*cloud_after_Radius);
    // std::cout << "半径滤波后点云数据点数：" << cloud_after_Radius->points.size() << std::endl;
}

void fitLine(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, Eigen::Vector3f &point, Eigen::Vector3f &direction)
{
    Eigen::MatrixXf points(cloud->points.size(), 2);
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        points(i, 0) = cloud->points[i].x;
        points(i, 1) = cloud->points[i].y;
    }

    Eigen::Vector2f centroid = points.colwise().mean();
    Eigen::MatrixXf centered = points.rowwise() - centroid.transpose();
    Eigen::Matrix2f covariance = centered.adjoint() * centered;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(covariance);

    point.head<2>() = centroid;
    direction.head<2>() = solver.eigenvectors().col(1).normalized();
    point.z() = 0;
    direction.z() = 0;
}

void Point2pgm::detectLines(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, 
    pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_line_opt)
{
    // 计算法向量
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(0.5);
    ne.compute(*normals);

    // 过滤掉天花板和地板的点，只保留墙体点
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZI>);
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        pcl::PointXYZI point = cloud->points[i];
        pcl::Normal normal = normals->points[i];

        // 判断法向量是否接近垂直于Z轴
        if (std::abs(normal.normal_z) < 0.4)
        {
            filteredCloud->points.push_back(point);
        }
    }

    // 计算点密度并过滤稀疏点
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(filteredCloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr remainingCloud(new pcl::PointCloud<pcl::PointXYZI>);

    float radius = 1.0;  // 搜索半径，根据实际情况调整
    int min_neighbors = 15;  // 最小邻居点数，低于此值的点将被移除

    for (size_t i = 0; i < filteredCloud->points.size(); ++i)
    {
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        if (kdtree.radiusSearch(filteredCloud->points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) >= min_neighbors)
        {
            pcl::PointXYZI point = filteredCloud->points[i];
            point.z = 0; // 投影到2D平面
            remainingCloud->points.push_back(point);
        }
    }
  // pcl::PointCloud<pcl::PointXYZI>::Ptr remainingCloud(new pcl::PointCloud<pcl::PointXYZI>(*cloud));
  //   // 将z值置为0，投影到2D平面
  //   for (auto &point : remainingCloud->points)
  //   {
  //       point.z = 0;
  //   }
    int lineCounter = 0;

    while (remainingCloud->size() > 50 && lineCounter < 2) // 设定一个阈值，当剩余点数少于这个值时停止检测
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr line2D(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f2D(new pcl::PointCloud<pcl::PointXYZI>);

        // 创建一个模型参数对象，用于记录结果
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        // inliers表示误差能容忍的点 记录点云的序号
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        // 创建一个分割器
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        // Optional
        seg.setOptimizeCoefficients(true);
        // Mandatory-设置目标几何形状
        seg.setModelType(pcl::SACMODEL_LINE); //SACMODEL_LINE SACMODEL_PLANE注：在这里面修改模型名称
        // 分割方法：随机采样法
        seg.setMethodType(pcl::SAC_RANSAC); //SAC_RANSAC
        // 最大的迭代的次数
        seg.setMaxIterations(100);
        // 设置误差容忍范围
        seg.setDistanceThreshold(0.2); // 注：填写具体数值
        // 输入点云
        seg.setInputCloud(remainingCloud); // 输入点云
        // 分割点云
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0)
        {
            std::cout << "Could not estimate a linear model for the given dataset." << std::endl;
            break;
        }

        // 获取点和删除点2D
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(remainingCloud); // 输入点云
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*line2D); // line2D本次分割出的2D线

        if (line2D->points.size() < 50)
            break;

        // 计算最优拟合直线参数
        Eigen::Vector3f point, direction;
        fitLine(line2D, point, direction);

        float x1 = point.x();
        float y1 = point.y();
        float dx = direction.x();
        float dy = direction.y();

        std::vector<float> t_values;
        double t_min = std::numeric_limits<double>::max();
        double t_max = -std::numeric_limits<double>::max();

        // 优化直线点，将它们拉到直线上
        for (size_t i = 0; i < line2D->points.size(); ++i)
        {
            float x0 = line2D->points[i].x;
            float y0 = line2D->points[i].y;

            float t = ((x0 - x1) * dx + (y0 - y1) * dy) / (dx * dx + dy * dy);

            line2D->points[i].x = x1 + t * dx;
            line2D->points[i].y = y1 + t * dy;
            line2D->points[i].z = 0;

            t_values.push_back(t);
            if (t < t_min) t_min = t;
            if (t > t_max) t_max = t;
        }

        // *cloud_line_opt += *line2D;

        // 对t值进行排序
        std::sort(t_values.begin(), t_values.end());

        // 递归检查t值的间隙并进行分段处理
        std::vector<std::pair<float, float>> segments;
        segments.push_back({t_min, t_max});
        for (size_t i = 1; i < t_values.size(); ++i)
        {
            double gap = t_values[i] - t_values[i - 1];
            if (gap > 1) // 这里的0.5是间隙阈值，可以根据需要调整
            {
                segments.back().second = t_values[i - 1];
                segments.push_back({t_values[i], t_max});
            }
        }

        std::cout<<"segments size: "<<segments.size()<<std::endl;

        int seg_cnt = 0;
        for (const auto& segment : segments)
        {
            float t_segment_min = segment.first;
            float t_segment_max = segment.second;

            pcl::PointXYZI min_point, max_point;
            min_point.x = x1 + t_segment_min * dx;
            min_point.y = y1 + t_segment_min * dy;
            min_point.z = 0;
            max_point.x = x1 + t_segment_max * dx;
            max_point.y = y1 + t_segment_max * dy;
            max_point.z = 0;

            // 沿着检测到的直线均匀取点，每隔0.1m
            pcl::PointCloud<pcl::PointXYZI>::Ptr interpolated_line(new pcl::PointCloud<pcl::PointXYZI>);
            float distance = std::sqrt((max_point.x - min_point.x) * (max_point.x - min_point.x) +
                                       (max_point.y - min_point.y) * (max_point.y - min_point.y));
            int num_interpolated_points = distance / 0.1; // 插值点的数量

            for (int i = 0; i <= num_interpolated_points; ++i)
            {
                float t = static_cast<float>(i) / num_interpolated_points;
                pcl::PointXYZI interpolated_point;
                interpolated_point.x = min_point.x + t * (max_point.x - min_point.x);
                interpolated_point.y = min_point.y + t * (max_point.y - min_point.y);
                interpolated_point.z = 0;
                interpolated_point.intensity = int8_t(100);
                interpolated_line->points.push_back(interpolated_point);
            }

            if (interpolated_line->points.size() < 10)
                continue;

            *cloud_line_opt += *interpolated_line;

            // // 添加直线到可视化
            // std::string cloud_name = "line" + std::to_string(lineCounter) + "_seg" + std::to_string(seg_cnt);
            // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> single_color(interpolated_line, 0, 255, 0);
            // viewer->addPointCloud<pcl::PointXYZI>(interpolated_line, single_color, cloud_name);

            // viewer->addLine(min_point, max_point, 1.0, 0.0, 0.0, "line_" + std::to_string(lineCounter) + "_seg" + std::to_string(seg_cnt));
        }

        // *cloud_line_opt += *line2D;

        // 删除直线点
        extract.setNegative(true);
        extract.filter(*cloud_f2D);
        *remainingCloud = *cloud_f2D; // 更新剩余的点云
        lineCounter++;
    }
    // *cloud_line_opt += *remainingCloud;
}


// Helper function to process a subset of the point cloud
void processPointCloudSubset(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const nav_msgs::Odometry& odom, 
                             int start_idx, int end_idx, 
                             double bound_down, double bound_right, double map_resolution, 
                             int map_width, int map_height, 
                             std::vector<int8_t>& map_data, cv::Mat& occupancy_grid) {
    double x1 = odom.pose.pose.position.x;
    double y1 = odom.pose.pose.position.y;

    for (int iter = start_idx; iter < end_idx; ++iter) {
        double x2 = cloud->points[iter].x;
        double y2 = cloud->points[iter].y;
        int intensity = std::abs(int(cloud->points[iter].intensity));
        if (intensity == 0) intensity = 100;
        if (intensity > 255) intensity = 255;
        if (intensity == 127 || intensity == 128) intensity = 129;
        intensity -= 128;

        double dx = x2 - x1;
        double dy = y2 - y1;
        double distance = std::sqrt(dx * dx + dy * dy);
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

        for (int step = 0; step <= steps; ++step) {
            int x = indices_x[step];
            int y = indices_y[step];
            if (x >= 0 && x < map_width && y >= 0 && y < map_height) {
                int value = (step < steps) ? 0 : intensity;
                if (map_data[y * map_width + x] != -1 && map_data[y * map_width + x] != 0)
                    break;
                else {
                    map_data[y * map_width + x] = value;
                    occupancy_grid.at<int8_t>(y, x) = value;
                }
            }
        }
    }
}

void Point2pgm::OccupancyMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const nav_msgs::Odometry& odom) {
    map_msg.header.stamp = ros::Time::now();
    map_msg.header.frame_id = "world";
    map_msg.info.map_load_time = ros::Time::now();

    if (cloud->points.empty()) {
        ROS_WARN("OccupancyMap points is empty!\n");
        return;
    }

    double x_min, x_max, y_min, y_max;
    // Code to calculate bounds...
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

    // Adjust map size and origin if necessary...
    if(x_max > bound_up) {
        bound_up += Offset_x;
        map_msg.info.width = int((bound_up - bound_down) / map_resolution);
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
        int Offset = Offset_y / map_resolution;
        map_msg.info.height = int((bound_left - bound_right) / map_resolution);
        map_msg.data.resize(map_msg.info.width * map_msg.info.height, -1);

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

    // Multithreading part
    const int num_threads = std::thread::hardware_concurrency(); // Get number of available threads
    const int cloud_size = cloud->points.size();
    const int points_per_thread = cloud_size / num_threads;

    std::vector<std::thread> threads;
    std::vector<int8_t> map_data_copy = map_msg.data; // Create a copy of map data for threads to work on

    for (int i = 0; i < num_threads; ++i) {
        int start_idx = i * points_per_thread;
        int end_idx = (i == num_threads - 1) ? cloud_size : start_idx + points_per_thread;

        threads.emplace_back(processPointCloudSubset, cloud, std::ref(odom), start_idx, end_idx,
                             bound_down, bound_right, map_resolution, 
                             map_msg.info.width, map_msg.info.height, 
                             std::ref(map_data_copy), std::ref(occupancy_grid));
    }

    for (auto& thread : threads) {
        thread.join(); // Wait for all threads to finish
    }

    map_msg.data = map_data_copy; // Update map data with processed results
}