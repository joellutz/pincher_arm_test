/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * 
 * Author: Michael Ferguson, Helen Oleynikova
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <actionlib/server/simple_action_server.h>
#include <turtlebot_block_manipulation/BlockDetectionAction.h>

#include <tf/transform_listener.h>

#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <cmath>
#include <algorithm>

/*const std::string arm_link = "/arm_base_link";
const double gripper_open = 0.04;
const double gripper_closed = 0.024;

const double z_up = 0.08;
const double z_down = -0.04;

const double block_size = 0.0127; */

//const std::string block_topic = "/turtlebot_blocks";

namespace turtlebot_block_manipulation
{

class BlockDetectionServer
{
private:
    
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<turtlebot_block_manipulation::BlockDetectionAction> as_;
  std::string action_name_;
  turtlebot_block_manipulation::BlockDetectionFeedback feedback_;
  turtlebot_block_manipulation::BlockDetectionResult result_;
  turtlebot_block_manipulation::BlockDetectionGoalConstPtr goal_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  
  tf::TransformListener tf_listener_;
  
  // Parameters from goal
  std::string arm_link;
  double block_size;
  double table_height;
  
  ros::Publisher block_pub_;
  
  // Parameters from node
  std::string block_topic;
  std::string pointcloud_topic;
  
public:
  BlockDetectionServer(const std::string name) : 
    nh_("~"), as_(nh_, name, false), action_name_(name)
  {
    // Load parameters from the server.
    nh_.param<std::string>("block_topic", block_topic, "/turtlebot_blocks");
    nh_.param<std::string>("pointcloud_topic", pointcloud_topic, "/camera/depth_registered/points");
  
    // Register the goal and feeback callbacks.
    as_.registerGoalCallback(boost::bind(&BlockDetectionServer::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&BlockDetectionServer::preemptCB, this));
    
    as_.start();
    
    // Subscribe to point cloud
    sub_ = nh_.subscribe(pointcloud_topic, 1, &BlockDetectionServer::cloudCb, this);
    pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("block_output", 1);
    
    block_pub_ = nh_.advertise< geometry_msgs::PoseArray >(block_topic, 1, true);
  }

  void goalCB()
  {
    ROS_INFO("[block detection] Received goal!");
    // accept the new goal
    result_.blocks.poses.clear();
    
    goal_ = as_.acceptNewGoal();
    
    block_size = goal_->block_size;
    table_height = goal_->table_height;
    arm_link = goal_->frame;
    
    result_.blocks.header.frame_id = arm_link;
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }

  void cloudCb ( const sensor_msgs::PointCloud2ConstPtr& msg )
  {
    // Only do this if we're actually actively working on a goal.
    if (!as_.isActive()) return;
    
    result_.blocks.header.stamp = msg->header.stamp;
    
    // convert to PCL
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg (*msg, cloud);
    
    // transform to whatever frame we're working in, probably the arm frame.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    tf_listener_.waitForTransform(std::string(arm_link), cloud.header.frame_id, cloud.header.stamp, ros::Duration(1.0));
    if (!pcl_ros::transformPointCloud (std::string(arm_link), cloud, *cloud_transformed, tf_listener_))
    {
      ROS_ERROR ("Error converting to desired frame");
      return;
    }

    // Create the segmentation object for the planar model and set all the parameters
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.01);
    
    // Limit to things we think are roughly at the table height.
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud_transformed); 
    pass.setFilterFieldName("z");
    
    pass.setFilterLimits(table_height - 0.05, table_height + block_size + 0.05);
    pass.filter(*cloud_filtered);
    if( cloud_filtered->points.size() == 0 ){
      ROS_ERROR("0 points left");
      return;
    }else
      ROS_INFO("Filtered, %d points left", (int) cloud_filtered->points.size());

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      return;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Write the planar inliers to disk
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_filtered);

    // Creating the KdTree object for the search method of the extraction
    pcl::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.01); // 2cm
    ec.setMinClusterSize (20);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud( cloud_filtered);
    ec.extract (cluster_indices);

    pub_.publish(cloud_filtered);

    // for each cluster, see if it is a block
    for (size_t c = 0; c < cluster_indices.size (); ++c)
    {  
      // find the outer dimensions of the cluster
      float xmin = 0; float xmax = 0; float ymin = 0; float ymax = 0;
      float zmin = 0; float zmax = 0;
      for (size_t i = 0; i < cluster_indices[c].indices.size(); i++)
      {
          int j = cluster_indices[c].indices[i];
          float x = cloud_filtered->points[j].x;
          float y = cloud_filtered->points[j].y;
          float z = cloud_filtered->points[j].z;
          if (i == 0)
          {
            xmin = xmax = x;
            ymin = ymax = y;
            zmin = zmax = z;
          }
          else
          {
            xmin = std::min(xmin, x);
            xmax = std::max(xmax, x);
            ymin = std::min(ymin, y);
            ymax = std::max(ymax, y);
            zmin = std::min(zmin, z);
            zmax = std::max(zmax, z);
          }    
      }
      
      // Check if these dimensions make sense for the block size specified
      float xside = xmax-xmin;
      float yside = ymax-ymin;
      float zside = zmax-zmin;
      
      const float tol = 0.01; // 1 cm error tolerance
      // In order to be part of the block, xside and yside must be between
      // blocksize and blocksize*sqrt(2)
      // z must be equal to or smaller than blocksize
      if (xside > block_size-tol && xside < block_size*sqrt(2)+tol &&
          yside > block_size-tol && yside < block_size*sqrt(2)+tol &&
          zside < block_size+tol)
      {
        // If so, then figure out the position and the orientation of the block
        float angle = asin(block_size/(yside));
        
        if (yside < block_size)
          angle = 0.0;
        
        ROS_INFO_STREAM("xside: " << xside << " yside: " << yside << " angle: " << angle);
        // Then add it to our set
        ROS_INFO("Adding a new block!");
        addBlock( xmin+(xside)/2.0, ymin+(yside)/2.0, zmax - block_size/2.0, angle);
      }
    }
    
    if (result_.blocks.poses.size() > 0)
    {
      as_.setSucceeded(result_);
      block_pub_.publish(result_.blocks);
    }
    //else
    //  as_.setAborted(result_);
  }
    
  void addBlock(float x, float y, float z, float angle)
  {
    geometry_msgs::Pose block_pose;
    block_pose.position.x = x;
    block_pose.position.y = y;
    block_pose.position.z = z;
    
    Eigen::Quaternionf quat(Eigen::AngleAxis<float>(angle, Eigen::Vector3f(0,0,1)));
    
    block_pose.orientation.x = quat.x();
    block_pose.orientation.y = quat.y();
    block_pose.orientation.z = quat.z();
    block_pose.orientation.w = quat.w();
    
    result_.blocks.poses.push_back(block_pose);
  }

};

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "block_detection_action_server");

  turtlebot_block_manipulation::BlockDetectionServer server("block_detection");
  ros::spin();

  return 0;
}
