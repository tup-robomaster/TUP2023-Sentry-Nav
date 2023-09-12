#ifndef __DLLNODE_HPP__
#define __DLLNODE_HPP__

#include <vector>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.hpp>

#include "tf2/transform_datatypes.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.h>
#include <vector>
#include <string>
#include "grid3d.hpp"
#include "dllsolver.hpp"
#include <time.h>

using std::isnan;

//Class definition
class DLLNode : public rclcpp::Node
{
public:
    //!Default contructor 
    DLLNode(std::string node_name) : 
    rclcpp::Node(node_name), m_grid3d(node_name, this), m_solver(m_grid3d)
    {
        //Declare Params.
        this->declare_parameter("in_cloud", std::string("/pointcloud"));
        this->declare_parameter("base_frame_id", std::string("base_link"));
        this->declare_parameter("odom_frame_id", std::string("odom"));

        this->declare_parameter("use_imu",false);
        this->declare_parameter("use_yaw_increments",false);        
        this->declare_parameter("update_rate",10.0);
        this->declare_parameter("initial_x", 0.0);
        this->declare_parameter("initial_y", 0.0);
        this->declare_parameter("initial_z", 0.0);
        this->declare_parameter("initial_a", 0.0);
        this->declare_parameter("update_min_d", 0.1);
        this->declare_parameter("update_min_a", 0.1);
        this->declare_parameter("update_min_time", 1.0);
        this->declare_parameter("initial_z_offset", 0.0);
        this->declare_parameter("align_method", 1);
        this->declare_parameter("solver_max_iter", 75);
        this->declare_parameter("solver_max_threads", 8);

        // Read node parameters
        this->get_parameter("in_cloud", m_inCloudTopic);
        this->get_parameter("base_frame_id", m_baseFrameId);
        this->get_parameter("odom_frame_id", m_odomFrameId);
        this->get_parameter("global_frame_id", m_globalFrameId);
        this->get_parameter("use_imu", m_use_imu);
        this->get_parameter("use_yaw_increments", m_useYawIncrements);
        std::cout<<"m_inCloudTopic:"<<m_inCloudTopic<<std::endl;
        m_roll_imu = m_pitch_imu = m_yaw_imu = 0.0;
        
        // Read DLL parameters
        this->get_parameter("update_rate", m_updateRate);
        this->get_parameter("initial_x", m_initX);
        this->get_parameter("initial_y", m_initY);
        this->get_parameter("initial_z", m_initZ);
        this->get_parameter("initial_a", m_initA);
        this->get_parameter("update_min_d", m_dTh);
        this->get_parameter("update_min_a", m_aTh);
        this->get_parameter("update_min_time", m_tTh);
        this->get_parameter("initial_z_offset", m_initZOffset);
        this->get_parameter("align_method", m_alignMethod);
        this->get_parameter("solver_max_iter", m_solverMaxIter);
        this->get_parameter("solver_max_threads", m_solverMaxThreads);
        
        // Init internal variables
        m_init = false;
        m_doUpdate = true;

        // Init TF
        m_tfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        m_tfListener = std::make_shared<tf2_ros::TransformListener>(*m_tfBuffer);
        m_tfBr = std::make_shared<tf2_ros::TransformBroadcaster>(this);


        // Compute trilinear interpolation map 
        //m_grid3d.computeTrilinearInterpolation(); /* Now we compute the approximation online */

        // Setup solver parameters
        m_solver.setMaxNumIterations(m_solverMaxIter);
        m_solver.setMaxNumThreads(m_solverMaxThreads);

        // Launch subscribers
        m_pcSub = this->create_subscription<sensor_msgs::msg::PointCloud2>(m_inCloudTopic,
                                                                            rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
                                                                            std::bind(&DLLNode::pointcloudCallback,
                                                                            this, std::placeholders::_1));
        m_initialPoseSub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("initial_pose",
                                                                                                    2,
                                                                                                    std::bind(&DLLNode::initialPoseReceived,
                                                                                                    this,
                                                                                                    std::placeholders::_1));
        if(m_use_imu)
            m_imuSub = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data",
                                                                        rclcpp::QoS(rclcpp::KeepLast(2)),
                                                                        std::bind(&DLLNode::imuCallback,
                                                                        this,
                                                                        std::placeholders::_1));
        m_pcPub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/recv_transformed", 1);

        // Time stamp for periodic update
        m_lastPeriodicUpdate = this->now();

        // Launch updater timer
        updateTimer =rclcpp::create_timer(this,
                                            this->get_clock(),
                                            rclcpp::Duration::from_seconds(1.f / m_updateRate),
                                            std::bind(&DLLNode::checkUpdateThresholds, this));
        // Initialize TF from odom to map as identity
        m_lastGlobalTf.setIdentity();
                
        // if(m_initX != 0 || m_initY != 0 || m_initZ != 0 || m_initA != 0)
        // {
        tf2::Transform pose;
        tf2::Vector3 origin(m_initX, m_initY, m_initZ);
        tf2::Quaternion q;
        q.setRPY(0,0,m_initA);

        pose.setOrigin(origin);
        pose.setRotation(q);
        
        setInitialPose(pose, this->now());
        m_init = true;
        is_first_odom_tf_exists = false;
        // }/
    }

    //!Default destructor
    ~DLLNode()
    {
    }

    bool sendLatestTransform()
    {
        // Publish current TF from odom to map
        geometry_msgs::msg::TransformStamped global_2_odom_tf;
        //Use current timestamp in order to avoid extrapolation into future error.
        global_2_odom_tf.header.stamp = this->now();
        global_2_odom_tf.header.frame_id = m_globalFrameId;
        global_2_odom_tf.child_frame_id = m_odomFrameId;
        global_2_odom_tf.transform.translation.x = m_lastGlobalTf.getOrigin().getX();
        global_2_odom_tf.transform.translation.y = m_lastGlobalTf.getOrigin().getY();
        global_2_odom_tf.transform.translation.z = m_lastGlobalTf.getOrigin().getZ();
        global_2_odom_tf.transform.rotation.x = m_lastGlobalTf.getRotation().getX();
        global_2_odom_tf.transform.rotation.y = m_lastGlobalTf.getRotation().getY();
        global_2_odom_tf.transform.rotation.z = m_lastGlobalTf.getRotation().getZ();
        global_2_odom_tf.transform.rotation.w = m_lastGlobalTf.getRotation().getW();
        m_tfBr->sendTransform(global_2_odom_tf);
        return true;
    }

    //! Check motion and time thresholds for AMCL update
    bool checkUpdateThresholds()
    {
        // If the filter is not initialized then exit
        if(!m_init)
            return false;
        sendLatestTransform();

        // Compute odometric translation and rotation since last update 
        tf2::Transform odomTf;
        geometry_msgs::msg::TransformStamped odomTf_msg;
        try
        {
            odomTf_msg = m_tfBuffer->lookupTransform(m_odomFrameId, m_baseFrameId, tf2::TimePoint());
            tf2::convert(odomTf_msg.transform, odomTf);
        }
        catch (tf2::TransformException ex)
        {
            RCLCPP_ERROR(this->get_logger(), "DLL error: %s", ex.what());
            return false;
        }
        rclcpp::Time timestamp = odomTf_msg.header.stamp;

        tf2::Transform T;
        if (!is_first_odom_tf_exists)
            T = odomTf;
        else
            T = m_lastOdomTf.inverse() * odomTf;
        
        // Check translation threshold
        if(T.getOrigin().length() > m_dTh)
        {
            RCLCPP_INFO(this->get_logger(), "Translation update");
            m_doUpdate = true;
            m_lastPeriodicUpdate = timestamp;
            return true;
        }
        // Check yaw threshold
        double yaw, pitch, roll;
        T.getBasis().getRPY(roll, pitch, yaw);
        if(fabs(yaw) > m_aTh)
        {
            RCLCPP_INFO(this->get_logger(), "Rotation update");
            m_doUpdate = true;
            m_lastPeriodicUpdate = timestamp;
            return true;
        }
        // Check time threshold
        if((timestamp - m_lastPeriodicUpdate).seconds() > m_tTh)
        {
            RCLCPP_INFO(this->get_logger(), "Periodic update");
            m_doUpdate = true;
            m_lastPeriodicUpdate = timestamp;
            return true;
        }

        sendLatestTransform();
        return false;
    }
                                           
private:
    void initialPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        // We only accept initial pose estimates in the global frame
        if(msg->header.frame_id != m_globalFrameId)
        {
            RCLCPP_WARN(this->get_logger(), "Ignoring initial pose in frame \"%s\"; initial poses must be in the global frame, \"%s\"",
            msg->header.frame_id.c_str(),
            m_globalFrameId.c_str());
            return;    
        }
        
        // Transform into the global frame
        tf2::Transform pose;
        tf2::convert(msg->pose.pose, pose);
        //RCLCPP_INFO(this->get_logger(), "Setting pose (%.6f): %.3f %.3f %.3f %.3f", this->now().toSec(), pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z(), getYawFromTf(pose));
        
        // Initialize the filter
        setInitialPose(pose, msg->header.stamp);
    }
    
    //! IMU callback
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) 
    {
        // std::cout<<"IMU RECV"<<std::endl;
        double r = m_roll_imu;
        double p = m_pitch_imu;
        double y = m_yaw_imu;
        auto o = msg->orientation;
        tf2::Quaternion q;
        tf2::convert(o, q);
        // std::cout<<"X: "<<q.getX()<<" Y: "<<q.getY()<<" Z: "<<q.getZ()<<" W: "<<q.getW()<<std::endl;
        tf2::Matrix3x3 M(q);
        M.getRPY(m_roll_imu, m_pitch_imu, m_yaw_imu);
        if (isnan(m_roll_imu) || isnan(m_pitch_imu) || isnan(m_yaw_imu)) 
        {
            m_roll_imu = r;
            m_pitch_imu = p;
            m_yaw_imu = y;
        }
    }

    //! 3D point-cloud callback
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
    {    
        // std::cout<<"PC RECV"<<std::endl;
        last_pc_timestamp = rclcpp::Time(cloud->header.stamp);
        static double lastYaw_imu = -1000.0;
        double deltaYaw_imu = 0;
    
        // If the filter is not initialized then exit
        if(!m_init)
            return;
            
        // Check if an update must be performed or not
        if(!m_doUpdate)
            return;

        // Compute odometric translation and rotation since last update 
        tf2::Transform odomTf;
        geometry_msgs::msg::TransformStamped odomTf_msg;
        try
        {
            odomTf_msg = m_tfBuffer->lookupTransform(m_odomFrameId, m_baseFrameId, cloud->header.stamp, rclcpp::Duration::from_seconds(0.5));
            tf2::convert(odomTf_msg.transform, odomTf);
        }
        catch (tf2::TransformException ex)
        {
            RCLCPP_ERROR(this->get_logger(), "%s",ex.what());
            return;
        }
        tf2::Transform mapTf;
        mapTf = m_lastGlobalTf * odomTf;

        // Pre-cache transform for point-cloud to base frame and transform the pc
        try
        {
            m_pclTf_msg = m_tfBuffer->lookupTransform(m_baseFrameId, cloud->header.frame_id, cloud->header.stamp, rclcpp::Duration::from_seconds(0.5));
            tf2::convert(m_pclTf_msg.transform, m_pclTf);
        }
        catch (tf2::TransformException ex)
        {
            RCLCPP_ERROR(this->get_logger(), "%s",ex.what());
            return;
        }
        //TODO: Do transform last to accelerate.
        sensor_msgs::msg::PointCloud2 baseCloud;
        pcl_ros::transformPointCloud(m_baseFrameId, m_pclTf, *cloud, baseCloud);


        pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);//申明滤波前后的点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
        // Convert to PCL data type
        pcl_conversions::toPCL(baseCloud, *cloud_blob);


        // 创建体素栅格下采样: 下采样的大小为10cm
        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;  //体素栅格下采样对象
        sor.setInputCloud (cloud_blob);             //原始点云
        sor.setLeafSize (0.05f, 0.05f, 0.05f);    // 设置采样体素大小
        sor.filter (*cloud_filtered_blob);        //保存
        // 转换为模板点云
        pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);
        std::vector<pcl::PointXYZ> downCloud_vec;
        // PointCloud2 to PointXYZ conevrsion, with range limits [0,1000]
        // PointCloud2 to PointXYZ conevrsion, with range limits [0,1000]
        PointCloud2_to_PointXYZ(cloud_filtered, downCloud_vec);

        sensor_msgs::msg::PointCloud2 m_pcMsg;
        pcl::toROSMsg(*cloud_filtered, m_pcMsg);
        m_pcMsg.header = cloud->header;
        m_pcMsg.header.frame_id = m_baseFrameId;
        m_pcPub->publish(m_pcMsg);

        // Get estimated position into the map
        double tx, ty, tz;
        tx = mapTf.getOrigin().getX();
        ty = mapTf.getOrigin().getY();
        tz = mapTf.getOrigin().getZ();
        // std::cout<<"Prior: "<<tx<<" , "<<ty<<" , "<<tz<<std::endl;
        // std::cout<<"DTX:"<<m_lastGlobalTf.getOrigin().getX()<<" : "<<m_lastGlobalTf.getOrigin().getY()<<" : " \ 
        //             <<m_lastGlobalTf.getOrigin().getZ()<<std::endl;

        // Get estimated orientation into the map
        double roll, pitch, yaw;
        if(m_use_imu)
        {
            // Get roll and pitch from IMU, yaw from TF
            double r, p;
            mapTf.getBasis().getRPY(r, p, yaw);   
            roll = m_roll_imu;
            pitch = m_pitch_imu;

            // Get yaw increment from IMU, if so required
            if(m_useYawIncrements)
            {
                if(lastYaw_imu < -100.0)
                    lastYaw_imu = m_yaw_imu;
                deltaYaw_imu = m_yaw_imu-lastYaw_imu;
                lastYaw_imu = m_yaw_imu;
            }
        }
        else
        {
            mapTf.getBasis().getRPY(roll, pitch, yaw);
        }
        // std::cout<<roll<<" "<<yaw<<" "<<pitch<<std::endl;

        // Launch DLL solver
        double a = yaw;
        if(m_use_imu && m_useYawIncrements)
            a = yaw+deltaYaw_imu;

        if(m_alignMethod == 1) // DLL solver(DLL Method need compensate)
        {
            // Tilt-compensate point-cloud according to roll and pitch
            std::vector<pcl::PointXYZ> points;
            float cr, sr, cp, sp, cy, sy, rx, ry;
            float r00, r01, r02, r10, r11, r12, r20, r21, r22;
            sr = sin(roll);
            cr = cos(roll);
            sp = sin(pitch);
            cp = cos(pitch);
            r00 = cp;     r01 = sp*sr;     r02 = cr*sp;
            r10 =  0;     r11 = cr;        r12 = -sr;

            r20 = -sp;    r21 = cp*sr;     r22 = cp*cr;
            points.resize(downCloud_vec.size());
            for(int i = 0; i < downCloud_vec.size(); i++) 
            {
                float x = downCloud_vec[i].x, y = downCloud_vec[i].y, z = downCloud_vec[i].z;
                points[i].x = x*r00 + y*r01 + z*r02;
                points[i].y = x*r10 + y*r11 + z*r12;
                points[i].z = x*r20 + y*r21 + z*r22;         
            }

            m_solver.solve(points, tx, ty, tz, a);
            yaw = a;
        }
        else
        {
            double imu_roll,imu_pitch;
            imu_roll = roll;
            imu_pitch = pitch;
            if(m_alignMethod == 2) // NDT solver
            {
                if (!m_grid3d.alignNDT(downCloud_vec, tx, ty, tz, roll, pitch, yaw))
                {
                    m_lastGlobalTf = tf2::Transform();
                    return;
                }
            }
            else if(m_alignMethod == 3) // ICP solver
            {
                if (!m_grid3d.alignICP(downCloud_vec, tx, ty, tz, roll, pitch, yaw))
                {
                    m_lastGlobalTf = tf2::Transform();
                    return;
                }
            }
            
            if (abs(roll - imu_roll) > 0.5 || abs(pitch - imu_pitch) > 0.5)
            {
                roll = imu_roll;
                pitch = imu_pitch;
            }
        }
        
        // Update global TF
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        m_lastGlobalTf = tf2::Transform(q, tf2::Vector3(tx, ty, tz));
        RCLCPP_INFO(this->get_logger(), "LastTF: X1: %f Y: %f Z: %f ,R: %f P: %f Y: %f",tx,ty,tz,roll,pitch,yaw);
        m_lastGlobalTf *= odomTf.inverse();
        // RCLCPP_INFO(this->get_logger(), "LastTF: X: %f Y: %f Z: %f ,R: %f P: %f Y: %f",tx,ty,tz,roll,pitch,yaw)
        // Update time and transform information
        m_lastOdomTf = odomTf;
        m_doUpdate = false;
    }
    
    //! Set the initial pose of the particle filter
    void setInitialPose(tf2::Transform initPose, rclcpp::Time timestamp)
    {
        // Extract TFs for future updates
        try
        {
            m_lastOdomTf_msg = m_tfBuffer->lookupTransform(m_odomFrameId, m_baseFrameId, timestamp, rclcpp::Duration(1.0));
            tf2::convert(m_lastOdomTf_msg.transform, m_lastOdomTf);
        }
        catch (tf2::TransformException ex)
        {
            RCLCPP_ERROR(this->get_logger(), "%s",ex.what());
            return;
        }

        // Get estimated orientation from IMU if available
        double roll, pitch, yaw;
        if(m_use_imu)
        {
            // Get roll and pitch from IMU, yaw from TF
            double r, p;
            m_lastOdomTf.getBasis().getRPY(r, p, yaw);   
            roll = m_roll_imu;
            pitch = m_pitch_imu;
        }
        else
            m_lastOdomTf.getBasis().getRPY(roll, pitch, yaw);
        
        // Get position information from pose 
        tf2::Vector3 t = initPose.getOrigin();
        yaw = getYawFromTf(initPose);
        
        // Update global TF
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        m_lastGlobalTf = tf2::Transform(q, tf2::Vector3(t.x(), t.y(), t.z()+m_initZOffset));
        m_lastGlobalTf *= m_lastOdomTf.inverse();

        // Prepare next iterations        
        m_doUpdate = true;
        m_init = true;
    }
    
    //! Return yaw from a given TF
    float getYawFromTf(tf2::Transform& pose)
    {
        double yaw, pitch, roll;
        
        pose.getBasis().getRPY(roll, pitch, yaw);
        
        return (float)yaw;
    }

    bool PointCloud2_to_PointXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr in, std::vector<pcl::PointXYZ> &out)
    {        
        out.clear();
        out.reserve(10000);
        for(int nIndex = 0; nIndex < in->points.size (); nIndex++)
        {
            float x,y,z,d2;
            x = in->points[nIndex].x;
            y = in->points[nIndex].y;
            z = in->points[nIndex].z;
            d2 = x*x + y*y + z*z;
            if(d2 > 1 && d2 < 10000)
                out.push_back(pcl::PointXYZ(in->points[nIndex]));            
        }
        return true;
    }

    //! Indicates if the filter was initialized
    bool m_init;
    //! Judge whether first odom tf exists 
    bool is_first_odom_tf_exists;
    //! Use IMU flag
    bool m_use_imu, m_useYawIncrements;

    geometry_msgs::msg::TransformStamped m_pclTf_msg;
    tf2::Transform m_pclTf;
    //! Particles roll and pich (given by IMU)
    double m_roll_imu, m_pitch_imu, m_yaw_imu;
    
    //! Filter initialization
    double m_initX, m_initY, m_initZ, m_initA, m_initZOffset;
        
    //! Thresholds and params for filter updating
    double m_dTh, m_aTh, m_tTh;
    geometry_msgs::msg::TransformStamped m_lastOdomTf_msg;
    tf2::Transform m_lastOdomTf;
    tf2::Transform m_lastGlobalTf;
    bool m_doUpdate;
    double m_updateRate;
    int m_alignMethod, m_solverMaxIter, m_solverMaxThreads;
    rclcpp::Time m_lastPeriodicUpdate;
    rclcpp::Time last_pc_timestamp;
        
    //! Node parameters
    std::string m_inCloudTopic;
    std::string m_baseFrameId;
    std::string m_odomFrameId;
    std::string m_globalFrameId;
    
    //! ROS msgs and data
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tfBr;
    std::shared_ptr<tf2_ros::Buffer> m_tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tfListener;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pcPub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_pcSub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_initialPoseSub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imuSub;
    rclcpp::TimerBase::SharedPtr updateTimer;
    
    //! 3D distance drid
    Grid3d m_grid3d;
        
    //! Non-linear optimization solver
    DLLSolver m_solver;
};

#endif


