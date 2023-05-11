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

#include "grid_map_demos/PCDToGridmapDemo.hpp"
namespace grid_map_demos
{

PCDToGridmapDemo::PCDToGridmapDemo()
: Node("pcd_to_gridmap_demo"),
  map_(grid_map::GridMap({"elevation"})),
  gridMapPclLoader(this->get_logger()),
  filterChain_("grid_map::GridMap")
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZ>);

    readParameters();
    gridMapPublisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
    "grid_map", rclcpp::QoS(1).transient_local());
    updateTimer = rclcpp::create_timer(this,
                                    this->get_clock(),
                                    rclcpp::Duration::from_seconds(1),
                                    std::bind(&PCDToGridmapDemo::timerCallback, this));
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

PCDToGridmapDemo::~PCDToGridmapDemo()
{
}

bool PCDToGridmapDemo::readParameters()
{
    this->declare_parameter("config_file_path", std::string());
    this->declare_parameter("filter_chain_parameter_name", std::string("filters"));
    this->declare_parameter("map_frame_id", std::string());
    this->declare_parameter("pcd_file_path", std::string());

    this->get_parameter("config_file_path", configFilePath_);
    this->get_parameter("filter_chain_parameter_name", filterChainParametersName_);
    this->get_parameter("map_frame_id", mapFrameId_);
    this->get_parameter("pcd_file_path", PCDFilePath_);

    gridMapPclLoader.loadParameters(configFilePath_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(PCDFilePath_, *cloud);
    cloud_src = cloud;
    // pcl::fromPCLPointCloud2(cloudBlob, *cloud_src);
    // std::cout<<"4"<<std::endl;
    return true;
}

void PCDToGridmapDemo::timerCallback()
{
    gridMapPclLoader.setInputCloud(cloud_src);
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
    auto message = grid_map::GridMapRosConverter::toMessage(outputMap);
    message->header.stamp = this->now();
    gridMapPublisher_->publish(std::move(message));
}

}  // namespace grid_map_demos
