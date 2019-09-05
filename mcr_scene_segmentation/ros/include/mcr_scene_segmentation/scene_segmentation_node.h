#ifndef SCENE_SEGMENTATION_NODE_H
#define SCENE_SEGMENTATION_NODE_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include "mcr_scene_segmentation/clustered_point_cloud_visualizer.h"
#include "mcr_scene_segmentation/bounding_box_visualizer.h"
#include <mcr_scene_segmentation/label_visualizer.h>
#include "mcr_scene_segmentation/cloud_accumulation.h"

#include <dynamic_reconfigure/server.h>
#include <mcr_scene_segmentation/SceneSegmentationConfig.h>
#include <mcr_perception_msgs/ObjectList.h>

using mcr::visualization::BoundingBoxVisualizer;
using mcr::visualization::ClusteredPointCloudVisualizer;
using mcr::visualization::LabelVisualizer;
using mcr::visualization::Color;

class SceneSegmentationNode
{
    private:
        ros::NodeHandle nh_;
        ros::Publisher pub_debug_;
        ros::Publisher pub_boxes_;
        ros::Publisher pub_object_list_;
        ros::Publisher pub_event_out_;
        ros::Publisher pub_poses_1;
        ros::Publisher pub_poses_2;
        ros::Publisher marker_pub;
        ros::Subscriber sub_cloud_;
        ros::Subscriber sub_event_in_;

        ros::ServiceClient recognize_service;

        dynamic_reconfigure::Server<mcr_scene_segmentation::SceneSegmentationConfig> server_;

        tf::TransformListener transform_listener_;

        SceneSegmentation scene_segmentation_;
        CloudAccumulation::UPtr cloud_accumulation_;

        BoundingBoxVisualizer bounding_box_visualizer_;
        ClusteredPointCloudVisualizer cluster_visualizer_;
        LabelVisualizer label_visualizer_;

        bool add_to_octree_; //Flag to select the use of OCTREE
        double threshold; //threshold to compare poses and bag them
        std::string frame_id_;
        int object_id_;
        double octree_resolution_;
        bool add_new_cloud_; //Flag to add new point cloud
        PointCloud::Ptr main_cloud; //Pointer to store the point cloud
        bool get_plane_once_; //Flag to create a plance only once
        PointCloud::ConstPtr plane_cloud;
        PointCloud::Ptr circle_cloud;
        bool perform_segmentation;
        std::vector< std::vector<geometry_msgs::PoseStamped> > mul_Poses;
        PointCloud::Ptr test;
        visualization_msgs::Marker marker;
        bool create_circles_;



    private:
        void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
        void eventCallback(const std_msgs::String::ConstPtr &msg);
        void config_callback(mcr_scene_segmentation::SceneSegmentationConfig &config, uint32_t level);
        geometry_msgs::PoseArray segment();
        void multiple_segmentations();
        geometry_msgs::PoseStamped getPose(const BoundingBox &box);
        std::vector<geometry_msgs::PoseStamped> generateMockupCirclePoses(double cx, double cy, double radius, double start_angle, double end_angle);
        PointCloud::Ptr pose_to_pointcloud(const PointCloud::Ptr circle_cloud,std::vector<geometry_msgs::PoseStamped> pose_list);
        std::vector< std::vector<geometry_msgs::PoseStamped> > trackPoses();
        // std::vector< std::vector<geometry_msgs::PoseStamped> > trackMultiplePoses();
    public:
        SceneSegmentationNode();
        virtual ~SceneSegmentationNode();
};

#endif /* SCENE_SEGMENTATION_NODE_H */
