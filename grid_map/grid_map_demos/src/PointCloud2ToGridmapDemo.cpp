/*
 * ImageToGridmapDemo.cpp
 *
 *  Created on: May 4, 2015
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#include <string>
#include <utility>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

#include "grid_map_demos/PointCloud2ToGridmapDemo.hpp"
namespace grid_map_demos
{

PointCloud2ToGridmapDemo::PointCloud2ToGridmapDemo()
: Node("pointcloud_to_gridmap_demo"),
  map_(grid_map::GridMap({"elevation"})),
  gridMapPclLoader(this->get_logger()),
  filterChain_("grid_map::GridMap")
{
    readParameters();
    tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
    pointCloud2Subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(pointCloud2Topic_,
                                                                                         rclcpp::QoS(rclcpp::QoS(1).best_effort()),
                                                                                        std::bind(&PointCloud2ToGridmapDemo::pointCloud2Callback,
                                                                                        this, std::placeholders::_1));

    gridMapPublisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
    "grid_map", rclcpp::QoS(1).transient_local());
    
    // Setup filter chain.
    if (filterChain_.configure(
        filterChainParametersName_, this->get_node_logging_interface(),
        this->get_node_parameters_interface()))
    {
        RCLCPP_INFO(this->get_logger(), "Filter chain configured.");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Could not configure the filter chain!");
        rclcpp::shutdown();
        return;
    }
}

PointCloud2ToGridmapDemo::~PointCloud2ToGridmapDemo()
{
}

bool PointCloud2ToGridmapDemo::readParameters()
{
    this->declare_parameter("pointcloud_topic", std::string("/stereo/points"));
    this->declare_parameter("map_frame_id", std::string());
    this->declare_parameter("config_file_path", std::string());
    this->declare_parameter("min_height", rclcpp::ParameterValue(0.0));
    this->declare_parameter("max_height", rclcpp::ParameterValue(1.0));
    this->declare_parameter("integration_time", rclcpp::ParameterValue(0.5));
    this->declare_parameter("filter_chain_parameter_name", std::string("filters"));
    this->get_parameter("pointcloud_topic", pointCloud2Topic_);
    this->get_parameter("map_frame_id", mapFrameId_);
    this->get_parameter("config_file_path", configFilePath_);
    this->get_parameter("min_height", minHeight_);
    this->get_parameter("max_height", maxHeight_);
    this->get_parameter("integration_time", integration_time_);
    this->get_parameter("filter_chain_parameter_name", filterChainParametersName_);

    gridMapPclLoader.loadParameters(configFilePath_);
    return true;
}

void PointCloud2ToGridmapDemo::pointCloud2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    sensor_msgs::msg::PointCloud2::SharedPtr msg_transformed = std::make_shared<sensor_msgs::msg::PointCloud2>(*msg);
    auto frame_id = msg->header.frame_id;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trans(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud_src);
    
    // std::vector<int> indices;
    // pcl::removeNaNFromPointCloud(*cloud_src,*cloud_filtered, indices);

	// pcl::RadiusOutlierRemoval<pcl::PointXYZ> sor;   
	// sor.setInputCloud(cloud_src);
	// sor.setRadiusSearch(0.5);
	// sor.setMinNeighborsInRadius(2);
	// sor.setNegative(false); 
	// sor.filter(*cloud_filtered);  

    // Added Cond filter.
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>());
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
            new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 2.0)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
            new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, -2.0)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
            new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, 3.0)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
            new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, -3.0)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
            new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, 3.0)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
            new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, -3.0)));
    condrem.setCondition(range_cond);
    condrem.setInputCloud(cloud_src);
    condrem.setKeepOrganized(true);
    // apply filter
    condrem.filter(*cloud_filtered);

    if (frame_id != mapFrameId_)
    {
        geometry_msgs::msg::TransformStamped tf_msg;
        try
        {
            tf_msg = tfBuffer_->lookupTransform(mapFrameId_, frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
            Eigen::Matrix4f sensor_to_odom = tf2::transformToEigen(tf_msg.transform).matrix().cast<float>();
            pcl::transformPointCloud(*cloud_filtered, *cloud_trans,sensor_to_odom);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "%s",ex.what());
            return;
        }
    }
    else
    {
        cloud_trans = cloud_filtered;
    }

    rclcpp::Time msg_time(msg->header.stamp);
    while (timestamp_deque_.size() != 0)
    {
        double dura = (msg_time.nanoseconds() - timestamp_deque_[0].nanoseconds()) * 1e-9;
        if (dura < integration_time_)
        {
            break;
        }
        else
        {
            timestamp_deque_.pop_front();
            pointcloud_deque_.pop_front();
        }
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_integrated(new pcl::PointCloud<pcl::PointXYZ>);
    // cloud_integrated = cloud_trans;
    timestamp_deque_.push_back(std::move(msg_time));
    pointcloud_deque_.push_back(std::move(cloud_trans));
    for (size_t i = 0; i < pointcloud_deque_.size(); i++)
    {
        (*cloud_integrated) += (*pointcloud_deque_[i]);
    }
    std::cout<<"Current buffer size:"<<pointcloud_deque_.size()<<std::endl;
    // std::cout<<cloud_src->size()<<std::endl;
    // if (cloud_src->size())
    gridMapPclLoader.setInputCloud(cloud_integrated);
    gridMapPclLoader.preProcessInputCloud();
    gridMapPclLoader.initializeGridMapGeometryFromInputCloud();
    gridMapPclLoader.addLayerFromInputCloud("elevation");
    map_ = gridMapPclLoader.getGridMap();
    map_.setFrameId(mapFrameId_);
    grid_map::GridMap outputMap;
    if (!filterChain_.update(map_, outputMap))
    {
        RCLCPP_ERROR(this->get_logger(), "Could not update the grid map filter chain!");
        return;
    }
    // Publish as grid map.
    // double timestamp=msg->header.stamp.sec + 1e-9 * msg->header.stamp.nanosec;
    // double stamp_cur =  1e-9 * this->now().nanoseconds();
    // std::cout<<"DT:"<<stamp_cur - timestamp<<std::endl;
    auto message = grid_map::GridMapRosConverter::toMessage(outputMap);
    message->header.stamp = msg->header.stamp;
    gridMapPublisher_->publish(std::move(message));
}

}  // namespace grid_map_demos
