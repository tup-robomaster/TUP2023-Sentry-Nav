/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2020, Samsung R&D Institute Russia
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *         Alexey Merzlyakov
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_costmap2d_plugin.html
 *********************************************************************/
#include "nav2_msg_costmap_plugin/msg_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace nav2_msg_costmap_plugin
{

MsgLayer::MsgLayer()
: last_min_x_(-std::numeric_limits<float>::max()),
  last_min_y_(-std::numeric_limits<float>::max()),
  last_max_x_(std::numeric_limits<float>::max()),
  last_max_y_(std::numeric_limits<float>::max())
{
}

// This method is called at the end of plugin initialization.
// It contains ROS parameter(s) declaration and initialization
// of need_recalculation_ variable.
void MsgLayer::onInitialize()
{
    auto node = node_.lock(); 
    declareParameter("map_topic", rclcpp::ParameterValue("grid_map"));
    declareParameter("global_frame", rclcpp::ParameterValue("odom"));
    declareParameter("map_time_decay", rclcpp::ParameterValue(0.2));
    grid_map_ = std::make_shared<grid_map::GridMap>();
    node->get_parameter(name_ + "." + "map_topic", map_topic_);
    node->get_parameter(name_ + "." + "global_frame", global_frame_);
    node->get_parameter(name_ + "." + "map_time_decay", time_decay_);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(rclcpp_node_);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    subscriber_ = node->create_subscription<grid_map_msgs::msg::GridMap>(map_topic_,
                                                                        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
                                                                        std::bind(&MsgLayer::callback,
                                                                            this, std::placeholders::_1));
    need_recalculation_ = false;
    current_ = true;
}


void MsgLayer::callback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
{
    auto t1 = std::chrono::steady_clock::now();
    auto frame_id = msg->header.frame_id;
    tf2::Transform transform;
    geometry_msgs::msg::TransformStamped tf_msg;
    try
    {
        tf_msg = tf_buffer_->lookupTransform(global_frame_, frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.2));
        tf2::convert(tf_msg.transform, transform);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR(rclcpp_node_->get_logger(), "%s",ex.what());
        return;
    }
    auto t2 = std::chrono::steady_clock::now();
    //Get euler angle of rotation and set a new transform only with yaw
    double roll,pitch,yaw;
    tf2::Vector3 origin;
    transform.getBasis().getRPY(roll,pitch,yaw);
    origin = transform.getOrigin();
    tf2::Quaternion q_projected;
    q_projected.setRPY(0,0,yaw);
    origin.setZ(0);
    map_lock_.lock();
    transform_projected_.setRotation(q_projected);
    transform_projected_.setOrigin(origin);
    geometry_msgs::msg::TransformStamped tf_projected_stamped;
    tf_projected_stamped.header.stamp = msg->header.stamp;
    tf_projected_stamped.header.frame_id = global_frame_;
    tf_projected_stamped.child_frame_id = global_frame_ + "_projected";
    tf_projected_stamped.transform.translation.x = transform_projected_.getOrigin().getX();
    tf_projected_stamped.transform.translation.y = transform_projected_.getOrigin().getY();
    tf_projected_stamped.transform.translation.z = transform_projected_.getOrigin().getZ();
    tf_projected_stamped.transform.rotation.x = transform_projected_.getRotation().x();
    tf_projected_stamped.transform.rotation.y = transform_projected_.getRotation().y();
    tf_projected_stamped.transform.rotation.z = transform_projected_.getRotation().z();
    tf_projected_stamped.transform.rotation.w = transform_projected_.getRotation().w();
    geometry_msgs::msg::TransformStamped tf_projected_stamped_inverse;
    tf_projected_stamped_inverse.transform.translation.x = transform_projected_.inverse().getOrigin().getX();
    tf_projected_stamped_inverse.transform.translation.y = transform_projected_.inverse().getOrigin().getY();
    tf_projected_stamped_inverse.transform.translation.z = transform_projected_.inverse().getOrigin().getZ();
    tf_projected_stamped_inverse.transform.rotation.x = transform_projected_.inverse().getRotation().x();
    tf_projected_stamped_inverse.transform.rotation.y = transform_projected_.inverse().getRotation().y();
    tf_projected_stamped_inverse.transform.rotation.z = transform_projected_.inverse().getRotation().z();
    tf_projected_stamped_inverse.transform.rotation.w = transform_projected_.inverse().getRotation().w();
    tf_broadcaster_->sendTransform(tf_projected_stamped);
    grid_map::GridMap grid_map;
    grid_map::GridMapRosConverter::fromMessage(*msg, grid_map);
    auto map_ptr = std::make_shared<grid_map::GridMap>(grid_map.getTransformedMap(tf2::transformToEigen(tf_projected_stamped.transform),
                                                                                                                "elevation", global_frame_));
    map_ptr->add("slope",grid_map.getTransformedMap(tf2::transformToEigen(tf_projected_stamped.transform),"slope", global_frame_)["slope"]);
    double timestamp=msg->header.stamp.sec + 1e-9 * msg->header.stamp.nanosec;
    // auto node = node_.lock();
    // double stamp_cur =  1e-9 * node->now().nanoseconds();
    // std::cout<<"DT:"<<stamp_cur - timestamp<<std::endl;
    //Erase Map out of time decay.
    std::vector<std::shared_ptr<grid_map::GridMap>> grid_map_vec_tmp;
    std::vector<double> grid_map_timestamp_tmp;

    for (int i=0; i<grid_map_timestamp.size(); i++)
    {
        if (abs(timestamp - grid_map_timestamp[i]) < time_decay_)
        {
            grid_map_vec_tmp.push_back(grid_map_vec[i]);
            grid_map_timestamp_tmp.push_back(grid_map_timestamp[i]);
        }
    }

    grid_map_vec = grid_map_vec_tmp;
    grid_map_timestamp = grid_map_timestamp_tmp;
    grid_map_ = map_ptr;
    auto t4 = std::chrono::steady_clock::now();
    if (grid_map_vec.size() > 0)
    {
        for (int i=0; i < grid_map_vec.size();i++)
        {
            grid_map_->addDataFrom(*grid_map_vec[i],true,false,true);
        }
    }
    grid_map_vec.emplace_back(map_ptr);
    grid_map_timestamp.push_back(timestamp);
    auto t3 = std::chrono::steady_clock::now();
    // std::cout<<"t1: "<<(float)(std::chrono::duration<double,std::milli>(t2 - t1).count())<<std::endl;
    map_lock_.unlock();

    //TODO: Mapping
}

// The method is called to ask the plugin: which area of costmap it needs to update.
// Inside this method window bounds are re-calculated if need_recalculation_ is true
// and updated independently on its value.
void MsgLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
    *min_x = robot_x - 3;
    *max_x = robot_x + 3;
    *min_y = robot_y - 3;
    *max_y = robot_y + 3;
    // // std::cout<<robot_x<<" : "<<robot_y<<" : "<<robot_yaw<<std::endl;
    // if (need_recalculation_)
    // {
    //     last_min_x_ = *min_x;
    //     last_min_y_ = *min_y;
    //     last_max_x_ = *max_x;
    //     last_max_y_ = *max_y;
    //     // For some reason when I make these -<double>::max() it does not
    //     // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
    //     // -<float>::max() instead.
    //     *min_x = -std::numeric_limits<float>::max();
    //     *min_y = -std::numeric_limits<float>::max();
    //     *max_x = std::numeric_limits<float>::max();
    //     *max_y = std::numeric_limits<float>::max();
    //     need_recalculation_ = false;
    // }
    // else
    // {
    //     double tmp_min_x = last_min_x_;
    //     double tmp_min_y = last_min_y_;
    //     double tmp_max_x = last_max_x_;
    //     double tmp_max_y = last_max_y_;
    //     last_min_x_ = *min_x;
    //     last_min_y_ = *min_y;
    //     last_max_x_ = *max_x;
    //     last_max_y_ = *max_y;
    //     *min_x = std::min(tmp_min_x, *min_x);
    //     *min_y = std::min(tmp_min_y, *min_y);
    //     *max_x = std::max(tmp_max_x, *max_x);
    //     *max_y = std::max(tmp_max_y, *max_y);
    // }
}

// The method is called when footprint was changed.
// Here it just resets need_recalculation_ variable.
void
MsgLayer::onFootprintChanged()
{
    RCLCPP_DEBUG(rclcpp::get_logger("nav2_costmap_2d"), "MsgLayer::onFootprintChanged(): num footprint points: %lu",
                                                                            layered_costmap_->getFootprint().size());
}

// The method is called when costmap recalculation is required.
// It updates the costmap within its window bounds.
// Inside this method the costmap msg is generated and is writing directly
// to the resulting costmap master_grid without any merging with previous layers.
void
MsgLayer::updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
    unsigned char * master_array = master_grid.getCharMap();
    unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();
    // std::cout<<min_i<<" "<<max_i<<" "<<min_j<<" "<<max_i<<std::endl;
    // {min_i, min_j} - {max_i, max_j} - are update-window coordinates.
    // These variables are used to update the costmap only within this window
    // avoiding the updates of whole area.
    //
    // Fixing window coordinates with map size if necessary.
    min_i = std::max(0, min_i);
    min_j = std::max(0, min_j);
    max_i = std::min(static_cast<int>(size_x), max_i);
    max_j = std::min(static_cast<int>(size_y), max_j);

    // Simply computing one-by-one cost per each cell
    map_lock_.lock();
    for (int j = min_j; j < max_j; j++)
    {
        // Reset gradient_index each time when reaching the end of re-calculated window
        // by OY axis.
        for (int i = min_i; i < max_i; i++)
        {
            double worldx, worldy;
            master_grid.mapToWorld(i,j,worldx,worldy);
            Eigen::Vector2d vec2d_map(worldx,worldy);
            Eigen::Array2i idx_map;
            int index = master_grid.getIndex(i, j);
            // if (vec2d_map[1] < 0.2 && vec2d_map[1] > -0.2)
            // {
            //     if (vec2d_map[0] < 1.2 && vec2d_map[0] > 1.0)
            //         master_array[index] = 254;
            // }
            if (grid_map_->isInside(vec2d_map))
            {
                double slope = grid_map_->atPosition("slope", vec2d_map);
                double elevation = grid_map_->atPosition("elevation", vec2d_map);
                if (!std::isnan(elevation) && !std::isnan(slope))
                {
                    double cost;
                    if (elevation < 0.1 && elevation > -0.5)
                        cost = 0;
                    else
                        cost = 254;
                    // double cost = slope;
                    // cost = slope / 1.5708 * 254;
                    // if (cost > 200)
                    //     cost = 254;
                    // else
                    //     cost = 0;
                    master_array[index] = cost;
                }
            }
        }
    }
    map_lock_.unlock();
}

}  // namespace nav2_msg_costmap_plugin

// This is the macro allowing a nav2_msg_costmap_plugin::MsgLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_msg_costmap_plugin::MsgLayer, nav2_costmap_2d::Layer)
