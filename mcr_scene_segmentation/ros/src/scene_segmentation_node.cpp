#include <mcr_scene_segmentation/scene_segmentation.h>
#include <mcr_scene_segmentation/scene_segmentation_node.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/centroid.h>
#include <pcl_ros/point_cloud.h>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <math.h>

#include <mcr_perception_msgs/BoundingBox.h>
#include <mcr_perception_msgs/BoundingBoxList.h>
#include <mcr_perception_msgs/ObjectList.h>
#include <mcr_perception_msgs/RecognizeObject.h>
#include "mcr_scene_segmentation/impl/helpers.hpp"
#include "mcr_scene_segmentation/bounding_box.h"

#include <Eigen/Dense>

#include "pcl_ros/transforms.h"

SceneSegmentationNode::SceneSegmentationNode(): nh_("~"), bounding_box_visualizer_("bounding_boxes", Color(Color::SEA_GREEN)),
                                                          cluster_visualizer_("tabletop_clusters"), label_visualizer_("labels", mcr::visualization::Color(mcr::visualization::Color::TEAL)),
                                                          add_new_cloud_(false), get_plane_once_(true),object_id_(0)
{
    //Publishers
    pub_debug_ = nh_.advertise<sensor_msgs::PointCloud2>("output", 1);
    pub_object_list_ = nh_.advertise<mcr_perception_msgs::ObjectList>("object_list", 1);
    pub_event_out_ = nh_.advertise<std_msgs::String>("event_out", 1);
    marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    // topic to publish poses of a circle_poses
    pub_poses_1 = nh_.advertise< geometry_msgs::PoseArray >("c_poses", 1);

    //Subscribers
    sub_event_in_ = nh_.subscribe("event_in", 1, &SceneSegmentationNode::eventCallback, this);

    //Configuration file
    dynamic_reconfigure::Server<mcr_scene_segmentation::SceneSegmentationConfig>::CallbackType f =
                            boost::bind(&SceneSegmentationNode::config_callback, this, _1, _2);
    server_.setCallback(f);

    //Services
    recognize_service = nh_.serviceClient<mcr_perception_msgs::RecognizeObject>("/mcr_perception/object_recognizer/recognize_object");
    recognize_service.waitForExistence(ros::Duration(5));
    if (!recognize_service.exists())
    {
        ROS_WARN("Object recognition service is not available. Will return 'unknown' for all objects");
    }

    //Point clouds
    nh_.param("octree_resolution", octree_resolution_, 0.05);
    cloud_accumulation_ = CloudAccumulation::UPtr(new CloudAccumulation(octree_resolution_));
}

SceneSegmentationNode::~SceneSegmentationNode()
{

}


void SceneSegmentationNode::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    //Change add_to_octree flag functionality to add_new_cloud_
    if (add_new_cloud_)
    {
        PointCloud::Ptr cloud(new PointCloud);
        //PointCloud::Ptr circle_cloud(new PointCloud);
        pcl::PCLPointCloud2 pc2;
        pcl_conversions::toPCL(*msg, pc2);
        pcl::fromPCLPointCloud2(pc2, *cloud);


        pcl_ros::transformPointCloud("base_link",
                                 *cloud,
                                  *cloud,
                                  transform_listener_
                              );

        frame_id_ = cloud->header.frame_id;
        if (add_to_octree_)
        {
            //Add point cloud to an octree
            cloud_accumulation_->addCloud(cloud);
            ROS_INFO("Cloud stored as octree");
        }else
        {
            // Save the point cloud
            main_cloud = cloud;
            ROS_INFO("Cloud stored in main cloud");
        }
        std_msgs::String event_out;
        add_new_cloud_ = false;
        // event_out.data = "e_add_cloud_stopped";
        pub_event_out_.publish(event_out);
    }

}

void SceneSegmentationNode::multiple_segmentations()
{
    mcr_perception_msgs::ObjectList object_list;
    geometry_msgs::PoseArray poses;

    while (perform_segmentation)
    {
        add_new_cloud_ = true;
        ROS_WARN("About to perform segmentation.");
        poses = segment();
        ROS_INFO_STREAM("Poses " <<poses.poses.size());
        ROS_WARN("Segmentation performed");

        // creating empty vectors based on the number of poses we get the first time
        if (mul_Poses.size() == 0)
        {
            for (size_t i = 0; i < poses.poses.size(); i++)
            {
                geometry_msgs::PoseStamped ps;
                ps.pose = poses.poses[i];
                std::vector<geometry_msgs::PoseStamped> new_vector;
                new_vector.push_back(ps);
                mul_Poses.push_back(new_vector);
            }
        }

        //visualizing the circle_poses
        // double threshold = 0.0000002;
        for(size_t j=0; j < poses.poses.size(); j++)
        {

            double least_dist = std::numeric_limits<double>::infinity();
            int match_id = 0;

            double x = poses.poses[j].position.x;
            double y = poses.poses[j].position.y;

            for (size_t k=0; k<mul_Poses.size(); k++)
            {
                double cur_dist = 0.0;
                cur_dist = pow(pow(mul_Poses[k][mul_Poses[k].size()-1].pose.position.x - x,2.0) + \
                                     pow(mul_Poses[k][mul_Poses[k].size()-1].pose.position.y - y,2.0), 0.5);
                if (cur_dist < least_dist && cur_dist <= threshold){
                    least_dist = cur_dist;
                    match_id = static_cast<int>(k);
                }
            }
            geometry_msgs::PoseStamped ps;
            ps.pose = poses.poses[j];
            mul_Poses[match_id].push_back(ps);
        }

        ros::spinOnce();
    }
}

geometry_msgs::PoseArray SceneSegmentationNode::segment()
{

    //PointCloud pointer
    PointCloud::Ptr circle_cloud(new PointCloud);
    PointCloud::Ptr cloud(new PointCloud);
    cloud->header.frame_id = frame_id_;
    //downsample and flatten the cloud data'

    if (add_to_octree_)
    {
        //Recollect the cloud from the octree
        cloud_accumulation_->getAccumulatedCloud(*cloud);
        ROS_INFO("Cloud loaded from octree");
    }else
    {
        //Recollet the cloud from the main_cloud
        cloud = main_cloud;
        ROS_INFO("Cloud loaded from main cloud");
    }


    //create vector of type PointCloud pointer
    std::vector<PointCloud::Ptr> clusters;
    //create vector of type BoundingBox
    std::vector<BoundingBox> boxes;

    if (get_plane_once_)
    {
        get_plane_once_ = false;
        //get plane once
        plane_cloud = scene_segmentation_.getPlane(cloud);
        ROS_INFO("obtained the plane");
    }

    // std::vector< std::vector<geometry_msgs::PoseStamped> > ";


    //Code to check mock poses
    // std::vector<geometry_msgs::PoseStamped> pose_list;
    // pose_list = generateMockupCirclePoses(2.2, 2.2, 2.0, 0.523, 1.309);
    // pose1.push_back(pose_list);
    //
    // pose_list = generateMockupCirclePoses(2.5, 2.5, 2.2, 0.54, 1.509);
    //
    // pose1.push_back(pose_list);
    //
    // pose_list = generateMockupCirclePoses(2.0, 2.0, 2.4, 0.55, 1.409);
    //
    // pose1.push_back(pose_list);

    // std::vector< std::vector<geometry_msgs::PoseStamped> > sep_poses;
    // sep_poses = trackPoses();
    // ROS_INFO("GOT THE POSES!!!!");
    // std::cout << "/* NUMBER OF LISTS = */" << size()
    double radius = 0;
    double center_x = 0;
    double center_y = 0;

    PointCloud::Ptr debug = scene_segmentation_.getBoxes(cloud, plane_cloud, clusters, boxes);

    marker.header.frame_id = "base_link";
    if (create_circles_ == true)
    {
        int counter = 0;
        ROS_ERROR_STREAM("*********************** Number of list of poses " <<mul_Poses.size());
        for (size_t it = 0; it < mul_Poses.size(); it++)
        {
             ROS_ERROR_STREAM("*********************** Mul_poses " <<mul_Poses[it].size());
            if (mul_Poses[it].size() > 20) // 20 is kishaan's magic number
            {
                counter += 1;
                test = pose_to_pointcloud(circle_cloud, mul_Poses[it]);
                //convert the pose to a point cloud for visualizing
                scene_segmentation_.getCircleParams(circle_cloud, radius, center_x, center_y);
                ROS_INFO("---------------Parameters of the circles collected-------------------------");
                //obtain the circle parameters for each of the poses in the pose list
                // visualization_msgs::Marker marker;
                //initialize an marker object
                // marker.header.frame_id = "base_link";
                marker.header.stamp = ros::Time::now();
                marker.action = visualization_msgs::Marker::ADD;
                marker.color.a = 1;
                if (it == 0)
                {

                  marker.color.r = 0.0;
                  marker.color.g = 0.0;
                  marker.color.b = 1.0;
                  marker.id = it;
                  marker.type = visualization_msgs::Marker::CYLINDER;
                  marker.pose.position.x = center_x;
                  marker.pose.position.y = center_y;
                  marker.pose.position.z = 0.6;
                  marker.scale.x = radius; // cause these take the diameter of the circle
                  marker.scale.y = radius;
                  marker.scale.z = 0.0001;
                  marker_pub.publish(marker);
                }

                else if(it == 1)
                {
                  marker.color.r = 0.0;
                  marker.color.g = 1.0;
                  marker.color.b = 0.0;
                  marker.id = it;
                  marker.type = visualization_msgs::Marker::CYLINDER;
                  marker.pose.position.x = center_x;
                  marker.pose.position.y = center_y;
                  marker.pose.position.z = 0.5;
                  marker.scale.x = radius; // cause these take the diameter of the circle
                  marker.scale.y = radius;
                  marker.scale.z = 0.0001;
                  marker_pub.publish(marker);
                }
                else if(it == 2)
                {
                  marker.color.r = 1.0;
                  marker.color.g = 0.0;
                  marker.color.b = 0.0;
                  marker.id = it;
                  marker.type = visualization_msgs::Marker::CYLINDER;
                  marker.pose.position.x = center_x;
                  marker.pose.position.y = center_y;
                  marker.pose.position.z = 0.4;
                  marker.scale.x = radius; // cause these take the diameter of the circle
                  marker.scale.y = radius;
                  marker.scale.z = 0.0001;
                  marker_pub.publish(marker);
                }
                else
                {
                  marker.color.r = 1.0;
                  marker.color.g = 1.0;
                  marker.color.b = 1.0;
                  marker.id = it;
                  marker.type = visualization_msgs::Marker::CYLINDER;
                  marker.pose.position.x = center_x;
                  marker.pose.position.y = center_y;
                  marker.pose.position.z = 0.3;
                  marker.scale.x = radius; // cause these take the diameter of the circle
                  marker.scale.y = radius;
                  marker.scale.z = 0.0001;
                  marker_pub.publish(marker);
                }
            }
        }
        if (counter == mul_Poses.size() && mul_Poses.size() != 0)
        {
            ROS_WARN("---------------No more markers-------------------------");
            create_circles_ = false;
        }

    }
    ROS_INFO("---------------Markers published-------------------------");
    //uncomment this line to see the output from the list of poses.
    // pub_debug_.publish(*test);

    //debug contains the filtered point cloud
    pub_debug_.publish(*debug);

    //message of type BoundingBoxList
    mcr_perception_msgs::BoundingBoxList bounding_boxes;
    //message of type ObjectList
    mcr_perception_msgs::ObjectList object_list;
    //message of type PoseArray
    geometry_msgs::PoseArray poses;

    //resizing bounding_boxes array to the size of boxes
    bounding_boxes.bounding_boxes.resize(boxes.size());
    //resizing object_list array to the size of boxes
    object_list.objects.resize(boxes.size());
    //creating a vector of type string
    std::vector<std::string> labels;

    ros::Time now = ros::Time::now();
    for (int i = 0; i < boxes.size(); i++)
    {
        //Convert from bounding box to ros messages
        //bounding_box_msg.vertices contains the resulted points
        convertBoundingBox(boxes[i], bounding_boxes.bounding_boxes[i]);

        //message of type PointCloud2 -- depreciated
        sensor_msgs::PointCloud2 ros_cloud;

        //message of type PointCloud2
        pcl::PCLPointCloud2 pc2;

        //converting the clusters to type PointCloud2
        pcl::toPCLPointCloud2(*clusters[i], pc2);
        pcl_conversions::fromPCL(pc2, ros_cloud);

        //object recognition service
        if (recognize_service.exists())
        {
            //empty service which outputs a string and probability
            mcr_perception_msgs::RecognizeObject srv;
            //giving the inputs to the service
            srv.request.cloud = ros_cloud;
            srv.request.dimensions = bounding_boxes.bounding_boxes[i].dimensions;
            //return the name and the probability of the recognized object in the bounding box
            if (recognize_service.call(srv))
            {
                object_list.objects[i].name = srv.response.name;
                object_list.objects[i].probability = srv.response.probability;
            }
            else
            //update dummy values and produce error message in case of service unavailability
            {
                ROS_WARN("Object recognition service call failed");
                object_list.objects[i].name = "unknown";
                object_list.objects[i].probability = 0.0;
            }
        }
        //update dummy values in case if the recognize_service doesn't exist
        else
        {
            ROS_INFO("Recognize service does not exist");
            object_list.objects[i].name = "unknown";
            object_list.objects[i].probability = 0.0;
        }
        //update the names and probabilities in the labels array
        labels.push_back(object_list.objects[i].name);

        //saving the reference coordinate frame and timestamp of the boxes
        geometry_msgs::PoseStamped pose = getPose(boxes[i]);
        pose.header.stamp = now;
        pose.header.frame_id = frame_id_;


        std::string target_frame_id;
        //transform the pose of the boxes to the desired target_frame_id
        if (nh_.hasParam("target_frame_id"))
        {
            nh_.param("target_frame_id", target_frame_id, frame_id_);
            if (target_frame_id != frame_id_)
            {
                try
                {
                    ros::Time common_time;
                    transform_listener_.getLatestCommonTime(frame_id_, target_frame_id, common_time, NULL);
                    pose.header.stamp = common_time;
                    transform_listener_.waitForTransform(target_frame_id, frame_id_, common_time, ros::Duration(0.1));
                    geometry_msgs::PoseStamped pose_transformed;
                    transform_listener_.transformPose(target_frame_id, pose, pose_transformed);
                    object_list.objects[i].pose = pose_transformed;
                }
                catch(tf::LookupException& ex)
                {
                    ROS_WARN("Failed to transform pose: (%s)", ex.what());
                    pose.header.stamp = now;
                    //update the transformed poses accordingly
                    object_list.objects[i].pose = pose;
                }
            }
            else
            {
                //update the transformed poses accordingly
                object_list.objects[i].pose = pose;
            }
        }
        else
        {
            //update the transformed poses accordingly
            object_list.objects[i].pose = pose;
        }
        //update the poseArray message with the transformed poses
        //NOTE: Is it possible to use this poses list already without extracting first the object list?
        poses.poses.push_back(object_list.objects[i].pose.pose);
        poses.header = object_list.objects[i].pose.header;

        //update the database_id for the object list
        object_list.objects[i].database_id = object_id_;
        object_id_++;
    }
    //publish the recognized list of objects and others for visualization
    pub_object_list_.publish(object_list);
    bounding_box_visualizer_.publish(bounding_boxes.bounding_boxes, frame_id_);
    cluster_visualizer_.publish<PointT>(clusters, frame_id_);
    label_visualizer_.publish(labels, poses);
    return poses;
}

geometry_msgs::PoseStamped SceneSegmentationNode::getPose(const BoundingBox &box)
{
    BoundingBox::Points vertices = box.getVertices();
    Eigen::Vector3f n1;
    Eigen::Vector3f n2;
    Eigen::Vector3f n3 = (vertices[4] - vertices[0]) / (vertices[4] - vertices[0]).norm();
    if ((vertices[1] - vertices[0]).norm() > (vertices[3] - vertices[0]).norm())
    {
        n1 = (vertices[1] - vertices[0]) / (vertices[1] - vertices[0]).norm();
    }
    else
    {
        n1 = (vertices[3] - vertices[0]) / (vertices[3] - vertices[0]).norm();
    }
    n2 = n3.cross(n1);
    ROS_INFO_STREAM("got norms");
    Eigen::Matrix3f m;
    m << n1 , n2 , n3;
    Eigen::Quaternion<float> q(m);
    q.normalize();


    Eigen::Vector3f centroid = box.getCenter();
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = centroid(0);
    pose.pose.position.y = centroid(1);
    pose.pose.position.z = centroid(2);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    return pose;
}

std::vector<geometry_msgs::PoseStamped> SceneSegmentationNode::generateMockupCirclePoses(double cx, double cy, double radius, double start_angle, double end_angle)
{
    std::vector<geometry_msgs::PoseStamped> poses;
    int num_samples = (end_angle - start_angle) / 0.03; //1500 samples
    double cur_angle = start_angle;

    const double mean = 0.5;
    const double stddev = 0.02;
    boost::mt19937 rng;
    boost::normal_distribution<> nd(0.5, 0.01);
    boost::variate_generator<boost::mt19937&,
                             boost::normal_distribution<> > var_nor(rng, nd);

    for (int i = 0; i < num_samples; i++)
    {
        cur_angle = cur_angle + 0.03;
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = cx + (radius * cos(cur_angle))  + double(var_nor());
        pose.pose.position.y = cy + (radius * sin(cur_angle))  + double(var_nor());
        pose.pose.orientation.w = 1.0;
        poses.push_back(pose);
     }
     return poses;
}

PointCloud::Ptr SceneSegmentationNode::pose_to_pointcloud(const PointCloud::Ptr circle_cloud, std::vector<geometry_msgs::PoseStamped> pose_list)
{


 circle_cloud->width  = pose_list.size();
 circle_cloud->height = 1;
 circle_cloud->points.resize (circle_cloud->width * circle_cloud->height);

 // Generate the data
 for (size_t i = 0; i < circle_cloud->points.size (); ++i)
 {
   circle_cloud->points[i].x = pose_list[i].pose.position.x;
   circle_cloud->points[i].y = pose_list[i].pose.position.y;
   circle_cloud->points[i].z = pose_list[i].pose.orientation.w;
 }
 return circle_cloud;
}

void SceneSegmentationNode::eventCallback(const std_msgs::String::ConstPtr &msg)
{
    std_msgs::String event_out;
    //Function used to perform certain actions based on the message obtained.
    ROS_WARN("Event requested");

    if(msg->data == "e_start")
    {
        sub_cloud_ = nh_.subscribe("input", 1, &SceneSegmentationNode::pointcloudCallback, this);
        get_plane_once_ = true;
        create_circles_ = true;
        event_out.data = "e_started";

	}
    else if(msg->data == "e_add_cloud_start")
    {
        //Flag to activate the storage of a new cloud.
        add_new_cloud_ = true;
        if (add_to_octree_){
            ROS_INFO("Testing the octree flag");
        }else{
            ROS_INFO("Testing the main cloud");
        }
    }
    else if(msg->data == "e_add_cloud_stop")
    {
        add_new_cloud_ = false;
        event_out.data = "e_add_cloud_stopped";
    }
    else if(msg->data == "e_segment")
    {
        segment();
        event_out.data = "e_done";
    }
    else if(msg->data == "e_mul_segment")
    {
        perform_segmentation = true;
        ROS_WARN("Multiple segmentations selected");
        event_out.data == "e_mul_seg_implemented";
        multiple_segmentations();
    }
    else if(msg->data == "e_stop_segment")
    {
        perform_segmentation = false;
        add_new_cloud_ = false;
        ROS_WARN("Multiple segmentations stopped");
        event_out.data = "e_segmented_stopped";

    }
    else if(msg->data == "e_reset")
    {
        if (add_to_octree_)
        {
            //Reset the accumulation of octree clouds
            cloud_accumulation_->reset();
        }
        else{
            //Reset the cloud stored.
            main_cloud.reset(new PointCloud);
        }
        event_out.data = "e_reset";
    }
    else if(msg->data == "e_stop")
    {
		sub_cloud_.shutdown();
		event_out.data = "e_stopped";
	}
    else
    {
        return;
    }
	pub_event_out_.publish(event_out);

}

void SceneSegmentationNode::config_callback(mcr_scene_segmentation::SceneSegmentationConfig &config, uint32_t level)
{
    scene_segmentation_.setVoxelGridParams(config.voxel_leaf_size, config.voxel_filter_field_name, config.voxel_filter_limit_min, config.voxel_filter_limit_max);
    scene_segmentation_.setPassthroughParams(config.passthrough_filter_field_name, config.passthrough_filter_limit_min, config.passthrough_filter_limit_max);
    scene_segmentation_.setNormalParams(config.normal_radius_search);
    scene_segmentation_.setSACParams(config.sac_max_iterations, config.sac_distance_threshold, config.sac_optimize_coefficients, config.sac_eps_angle, config.sac_normal_distance_weight);
    scene_segmentation_.setPrismParams(config.prism_min_height, config.prism_max_height);
    scene_segmentation_.setOutlierParams(config.outlier_radius_search, config.outlier_min_neighbors);
    scene_segmentation_.setClusterParams(config.cluster_tolerance, config.cluster_min_size, config.cluster_max_size, config.cluster_min_height, config.cluster_max_height, config.cluster_max_length, config.cluster_min_distance_to_polygon);
    add_to_octree_ = config.add_to_octree_flag;
    threshold = config.pose_threshold;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "scene_segmentation_node");
    SceneSegmentationNode scene_seg;
    ros::spin();
    return 0;
}
