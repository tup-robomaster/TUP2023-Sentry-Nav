/*
 * PointCloud2ToGridmapDemo.hpp
 *
 *  Created on: Jan 9, 2022
 *      Author: Hao Gu
 *	 Institute: Shenyang Aerospace University
 *
 */

#ifndef GRID_MAP_DEMOS__POINTCLOUD2TOGRIDMAPDEMO_HPP_
#define GRID_MAP_DEMOS__POINTCLOUD2TOGRIDMAPDEMO_HPP_


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
class PointCloud2ToGridmapDemo : public rclcpp::Node
{
public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  PointCloud2ToGridmapDemo();

  /*!
   * Destructor.
   */
  virtual ~PointCloud2ToGridmapDemo();

  /*!
  * Reads and verifies the ROS parameters.
  * @return true if successful.
  */
  bool readParameters();

  void pointCloud2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

private:
  //! Grid map publisher.
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr gridMapPublisher_;

  //! Grid map data.
  grid_map::GridMap map_;

  //! GridMapPclLoader
  grid_map::GridMapPclLoader gridMapPclLoader;

  //! PointCloud2 subscriber
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloud2Subscriber_;

  //! Name of the grid map topic.
  std::string pointCloud2Topic_;

  //! Resolution of the grid map.
  double resolution_;

  //! Range of the height values.
  double minHeight_;
  double maxHeight_;

  //! Frame id of the grid map.
  std::string mapFrameId_;

  //! Path to config file.
  std::string configFilePath_;

  //! TF Buffer
  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;

  //! TF Listener
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;

  //! Filter chain.
  filters::FilterChain<grid_map::GridMap> filterChain_;

  //! Filter chain parameters name.
  std::string filterChainParametersName_;
};

}  // namespace grid_map_demos
#endif  // GRID_MAP_DEMOS__POINTCLOUD2TOGRIDMAPDEMO_HPP_
