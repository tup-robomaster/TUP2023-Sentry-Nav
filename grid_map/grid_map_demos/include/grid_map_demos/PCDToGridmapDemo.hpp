/*
 * PointCloud2ToGridmapDemo.hpp
 *
 *  Created on: Jan 9, 2022
 *      Author: Hao Gu
 *	 Institute: Shenyang Aerospace University
 *
 */

#ifndef GRID_MAP_DEMOS__PCDTOGRIDMAPDEMO_HPP_
#define GRID_MAP_DEMOS__PCDTOGRIDMAPDEMO_HPP_


#include <filters/filter_chain.hpp>
//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

//GRID_MAP_PCL
#include <grid_map_pcl/GridMapPclLoader.hpp>

//TF
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>
#include <vector>
namespace grid_map_demos
{

/*!
 * Loads an PointCloud2 and saves it as layer 'elevation' of a grid map.
 * The grid map is published and can be viewed in Rviz.
 */
class PCDToGridmapDemo : public rclcpp::Node
{
public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  PCDToGridmapDemo();

  /*!
   * Destructor.
   */
  virtual ~PCDToGridmapDemo();

  /*!
  * Reads and verifies the ROS parameters.
  * @return true if successful.
  */
  bool readParameters();

  void timerCallback();

private:
  //! Grid map publisher.
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr gridMapPublisher_;

  //! Grid map src.
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src;

  rclcpp::TimerBase::SharedPtr updateTimer;

  //! Grid map data.
  grid_map::GridMap map_;


  //! Frame id of the grid map.
  std::string mapFrameId_;

  //! Path to PCD file.
  std::string PCDFilePath_;
  
  //! GridMapPclLoader
  grid_map::GridMapPclLoader gridMapPclLoader;

  //! Path to config file.
  std::string configFilePath_;

  //! Filter chain.
  filters::FilterChain<grid_map::GridMap> filterChain_;

  //! Filter chain parameters name.
  std::string filterChainParametersName_;
};

}  // namespace grid_map_demos
#endif  // GRID_MAP_DEMOS__POINTCLOUD2TOGRIDMAPDEMO_HPP_
