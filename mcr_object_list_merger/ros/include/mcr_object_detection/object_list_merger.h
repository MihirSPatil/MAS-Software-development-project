/*
 * Copyright 2016 Bonn-Rhein-Sieg University
 *
 * Author: Santosh Thoduka
 *
 */

#ifndef MCR_OBJECT_DETECTION_OBJECT_LIST_MERGER_H
#define MCR_OBJECT_DETECTION_OBJECT_LIST_MERGER_H

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <mcr_perception_msgs/ObjectList.h>
#include <mir_knowledge_base_analyzer/ObjectsAtLocation.h>
#include <fstream>

/**
 * This node subscribes to ObjectList messages and merges them into a single message.
 * If objects from different lists are close to each other (specified by distance_threshold
 * parameter), the object with higher probability is chosen.
 *
 * Inputs:
 *  ~input_object_list: input object list
 *  ~event_in:
 *     - e_start: clears existing lists and starts listening to new object list messages
 *     - e_stop: stops listening to new object list messages
 *     - e_trigger: publishes merged list
 *
 * Outputs:
 * ~output_object_list: merged object list
 * ~event_out:
 *     - e_started: started listening to new messages
 *     - e_stopped: stopped listening to new messages
 *     - e_done:    published merged object list
 *     - e_empty_list: merged list is empty
 */
class ObjectListMerger
{
    public:
        ObjectListMerger();
        virtual ~ObjectListMerger();
        void update();
    private:
        void objectListCallback(const mcr_perception_msgs::ObjectList::Ptr &msg);
        void eventInCallback(const std_msgs::String::Ptr &msg);
        void objectsAtLocationCallback(const mir_knowledge_base_analyzer::ObjectsAtLocation::Ptr &msg);
        void mergeList(const mcr_perception_msgs::ObjectList::Ptr &new_object_list);
        double getDistance(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2);

        void updateMergedListWithPrior(mcr_perception_msgs::ObjectList *object_list);
        void updateMergedRGBListWithPrior(mcr_perception_msgs::ObjectList * object_list);
        void combineClassifierResults(mcr_perception_msgs::ObjectList &object_list);
        void writeObjectList(const mcr_perception_msgs::ObjectList &object_list);

    private:
        ros::NodeHandle nh_;

        ros::Publisher pub_object_list_;
        ros::Publisher pub_event_out_;

        ros::Publisher pub_query_;
        ros::Subscriber sub_objects_at_location_;
        //
        //ros::Subscriber sub_query_event_out_;

        ros::Subscriber sub_object_list_;
        ros::Subscriber sub_event_in_;

        std::string event_in_;

        bool event_in_received_;
        bool accepting_object_lists_;

        mcr_perception_msgs::ObjectList merged_object_list_;

        mir_knowledge_base_analyzer::ObjectsAtLocation objects_at_location_;

        mcr_perception_msgs::ObjectList rgb_object_list_;
        mcr_perception_msgs::ObjectList pointcloud_object_list_;

        std::vector<std::vector<std::string> > confused_groups_;
        std::map<std::string, int> confused_group_id_;

        double distance_threshold_;
        // if two objects within the distance_threshold have the same name,
        // update the pose to be the average pose (in x,y only)
        bool use_average_pose_;

        std::ofstream fout;
};

#endif  // MCR_OBJECT_DETECTION_OBJECT_LIST_MERGER_H
