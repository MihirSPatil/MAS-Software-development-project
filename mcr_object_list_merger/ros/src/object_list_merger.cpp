/*
 * Copyright 2016 Bonn-Rhein-Sieg University
 *
 * Author: Santosh Thoduka
 *
 */

#include <mcr_object_detection/object_list_merger.h>
#include <cmath>
#include <algorithm>
#include <strings.h>

ObjectListMerger::ObjectListMerger() : nh_("~"), event_in_received_(false), accepting_object_lists_(false), fout("/tmp/object_merger_results", std::ofstream::out | std::ofstream::app)
{
    nh_.param<double>("distance_threshold", distance_threshold_, 0.02);
    nh_.param<bool>("use_average_pose", use_average_pose_, false);
    pub_object_list_ = nh_.advertise<mcr_perception_msgs::ObjectList>("output_object_list", 1);
    pub_event_out_ = nh_.advertise<std_msgs::String>("event_out", 1);

    pub_query_ = nh_.advertise<std_msgs::String>("knowledge_base_query_topic", 1);

    sub_object_list_ = nh_.subscribe("input_object_list", 1, &ObjectListMerger::objectListCallback, this);
    sub_event_in_ = nh_.subscribe("event_in", 1, &ObjectListMerger::eventInCallback, this);

    sub_objects_at_location_ = nh_.subscribe("knowledge_base_objects_list", 1, &ObjectListMerger::objectsAtLocationCallback, this);
    //sub_query_event_out_ = nh_.subscribe("knowledge_base_event_out", 1, &ObjectListMerger::knowledgeBaseEventCallback, this);
    
    std::vector<std::string> g1;
    g1.push_back("S40_40_B");
    g1.push_back("MOTOR");
    confused_group_id_["S40_40_B"] = 0;
    confused_group_id_["MOTOR"] = 0;

    std::vector<std::string> g2;
    g2.push_back("F20_20_B");
    g2.push_back("M20_100");
    confused_group_id_["F20_20_B"] = 1;
    confused_group_id_["M20_100"] = 1;

    std::vector<std::string> g3;
    g2.push_back("F20_20_G");
    g2.push_back("AXIS");
    confused_group_id_["F20_20_G"] = 2;
    confused_group_id_["AXIS"] = 2;

    std::vector<std::string> g4;
    g3.push_back("BEARING_BOX");
    g3.push_back("M30");
//    g3.push_back("Motor");
    confused_group_id_["BEARING_BOX"] = 3;
    confused_group_id_["M30"] = 3;

    std::vector<std::string> g5;
    g4.push_back("BEARING");
    g4.push_back("M20");
    g4.push_back("DISTANCE_TUBE");
    confused_group_id_["BEARING"] = 4;
    confused_group_id_["M20"] = 4;
    confused_group_id_["DISTANCE_TUBE"] = 4;


    confused_groups_.push_back(g1);
    confused_groups_.push_back(g2);
    confused_groups_.push_back(g3);
    confused_groups_.push_back(g4);
    confused_groups_.push_back(g5);
}

ObjectListMerger::~ObjectListMerger()
{
    fout.close();
}

void ObjectListMerger::update()
{
    if (event_in_received_)
    {
        std_msgs::String event_out;
        if (event_in_ == "e_trigger")
        {
            mcr_perception_msgs::ObjectList * combined_recognition_list; 
            updateMergedRGBListWithPrior(&rgb_object_list_);
            fout << std::endl;
            fout << "RGB Combined and Replaced : " ;
            writeObjectList(rgb_object_list_);
            fout << std::endl;
            fout << "Pose based Map : " ;
            mcr_perception_msgs::ObjectList rgb_pointcloud_combined_list;
            if (!pointcloud_object_list_.objects.empty() and 
                !rgb_object_list_.objects.empty())
            {
                combineClassifierResults(rgb_pointcloud_combined_list);
                combined_recognition_list = &rgb_pointcloud_combined_list;
                //combined_recognition_list = &pointcloud_object_list_;
                // combine
                //
            }
            else if (!pointcloud_object_list_.objects.empty())
            {
                combined_recognition_list = &pointcloud_object_list_;
            }
            else
            {
                combined_recognition_list = &rgb_object_list_;
            } 

            fout << std::endl;
            fout << "Combined : " ;
            writeObjectList(*combined_recognition_list);

            if (!combined_recognition_list->objects.empty())
            {
                if (!objects_at_location_.objects.empty())
                {
                    updateMergedListWithPrior(combined_recognition_list);
                }
                fout << "Final published list : " ;
                writeObjectList(*combined_recognition_list);
                pub_object_list_.publish(*combined_recognition_list);
                event_out.data = "e_done";
            }
            else
            {
                event_out.data = "e_empty_list";
            }
            fout << "#####################################";
            fout << std::endl << std::endl << std::endl;
        }
        else if (event_in_ == "e_stop")
        {
            accepting_object_lists_ = false;
            event_out.data = "e_stopped";
        }
        else if (event_in_ == "e_start")
        {
            accepting_object_lists_ = true;
            merged_object_list_.objects.clear();
            objects_at_location_.objects.clear();
            objects_at_location_.location = "";
            rgb_object_list_.objects.clear();
            pointcloud_object_list_.objects.clear();
            event_out.data = "e_started";
            std_msgs::String query;
            query.data = "get_objects_at_current_location";
            pub_query_.publish(query);
        }
        event_in_received_ = false;
        pub_event_out_.publish(event_out);
    }
}

void ObjectListMerger::mergeList(const mcr_perception_msgs::ObjectList::Ptr &new_object_list)
{
    std::string type_of_list = "none";
    writeObjectList(*new_object_list);
    for (int i = 0; i < new_object_list->objects.size(); i++)
    {
        bool merged = false;
        const mcr_perception_msgs::Object new_object = new_object_list->objects[i];
        // if database id is > 99 we know the source is the RGB recognition, otherwise 3D recognition
        mcr_perception_msgs::ObjectList &list_to_merge = (new_object.database_id > 99) ? rgb_object_list_ : pointcloud_object_list_;
        type_of_list = (new_object.database_id > 99) ? "rgb list" : "pointcloud list";
        for (int j = 0; j < list_to_merge.objects.size(); j++)
        {
            mcr_perception_msgs::Object old_object = list_to_merge.objects[j];
            double distance = getDistance(new_object.pose.pose.position, old_object.pose.pose.position);
            // if both objects are close together
            if (distance < distance_threshold_)
            {
                ROS_INFO_STREAM("New object " << new_object.name << " is close to " << old_object.name);
                // if object names are the same, and use_average_pose is set
                // just change the pose
                if (use_average_pose_ && old_object.name == new_object.name)
                {
                    ROS_INFO_STREAM("Updating pose of " << old_object.name);
                    old_object.pose.pose.position.x =
                        (old_object.pose.pose.position.x + new_object.pose.pose.position.x) / 2;
                    old_object.pose.pose.position.y =
                        (old_object.pose.pose.position.y + new_object.pose.pose.position.y) / 2;
                }
                // if classification of new object has higher probabliity
                // use the new classification
                else if (new_object.probability > old_object.probability)
                {
                    ROS_INFO_STREAM("Replacing "
                                     << old_object.name << " with " << new_object.name);
                    old_object.name = new_object.name;
                }
                merged = true;
                break;
            }
        }
        if (!merged)
        {
            list_to_merge.objects.push_back(new_object);
        }
    }
//    ROS_INFO_STREAM("Total objects in " << type_of_list << ": " << list_to_merge.objects.size());
}

/*
 * Mering RGB list based on Inventory
 * RGB recognition has not learning about S40_40 and M30 because of similarity with F20_20 and M20
 */
void ObjectListMerger::updateMergedRGBListWithPrior(mcr_perception_msgs::ObjectList * object_list)
{
    // check if multiple F20_20_G in rgb list -> False fo nothing 
    // +--- check if S40_40_G in inventory -> False do nothing 
    // +------ Replace
    // check if multiple M20 in rgb list -> False fo nothing 
    // +--- check if M30 in inventory -> False do nothing 
    // +------ Replace
    // check if multiple F20_20_G or single R20 in rgb list -> False fo nothing 
    // +--- check if S40_40_B in inventory -> False do nothing 
    // +------ Replace
    std::string f20_20_g = "F20_20_G";
    std::string f20_20_b = "F20_20_B";
    std::string m20 = "M20";

    std::vector<std::string> inventory =  objects_at_location_.objects;
    for (int i = 0;i < object_list->objects.size(); i++)
    {
        std::string rgb_obj_name = object_list->objects[i].name;
        if (rgb_obj_name == f20_20_g)
        {
            int obj_count = std::count(inventory.begin(), inventory.end(), f20_20_g);
            if ((0 == obj_count) and (std::find(inventory.begin(), inventory.end(), "S40_40_G") != inventory.end()) )
            {
                object_list->objects[i].name = "S40_40_G";
                ROS_INFO_STREAM("RGB list Replacing " << f20_20_g << " with " << "S40_40_G" << " due to inventory");
            }
        }
        if (rgb_obj_name == f20_20_b)
        {
            int obj_count = std::count(inventory.begin(), inventory.end(), f20_20_b);
            if ((0 == obj_count) and (std::find(inventory.begin(), inventory.end(), "S40_40_B") != inventory.end()) )
            {
                object_list->objects[i].name = "S40_40_B";
                ROS_INFO_STREAM("RGB list Replacing " << f20_20_b << " with " << "S40_40_B" << " due to inventory");
            }
        }
        if (rgb_obj_name == m20)
        {
            int obj_count = std::count(inventory.begin(), inventory.end(), m20);

            if ((0 == obj_count) and (std::find(inventory.begin(), inventory.end(), "M30") != inventory.end()) )
            {
                object_list->objects[i].name = "M30";
                ROS_INFO_STREAM("RGB list Replacing " << m20 << " with " << "M30" << " due to inventory");
            }
        }
    }

}
void ObjectListMerger::updateMergedListWithPrior(mcr_perception_msgs::ObjectList * object_list)
{
    // if inventory has only one object, and recognized objects has only one object
    if (objects_at_location_.objects.size() == 1 and object_list->objects.size() == 1)
    {
        ROS_INFO_STREAM("Inventory has only one object: " << objects_at_location_.objects[0]);
        object_list->objects[0].name = objects_at_location_.objects[0];
        return;
    }

    std::vector<std::vector<std::string> > inv_groups;
    std::vector<std::vector<std::string> > rec_groups;
    // five confusion groups
    for (int i = 0; i < 5;i++)
    {
        std::vector<std::string> inv;
        std::vector<std::string> rec;
        inv_groups.push_back(inv);
        rec_groups.push_back(rec);
    }

    for (int i = 0;i < objects_at_location_.objects.size(); i++)
    {
        std::string obj = objects_at_location_.objects[i];
        if (confused_group_id_.find(obj) != confused_group_id_.end())
        {
            inv_groups[confused_group_id_[obj]].push_back(obj);
        }
    }

    for (int i = 0;i < object_list->objects.size(); i++)
    {
        std::string obj = object_list->objects[i].name;
        std::transform(obj.begin(), obj.end(), obj.begin(), ::toupper);
        if (confused_group_id_.find(obj) != confused_group_id_.end())
        {
            rec_groups[confused_group_id_[obj]].push_back(obj);
        }
    }

    for (int i = 0;i < object_list->objects.size(); i++)
    {
        std::string obj = object_list->objects[i].name;
        std::transform(obj.begin(), obj.end(), obj.begin(), ::toupper);
        ROS_INFO_STREAM("Checking " <<  obj);
        if (confused_group_id_.find(obj) == confused_group_id_.end())
        {
            continue;
        }
        int group = confused_group_id_[obj];
        if (rec_groups[group].size() == 1 && inv_groups[group].size() == 1 
            && obj != inv_groups[group][0])
        {
            object_list->objects[i].name = inv_groups[group][0];
            ROS_INFO_STREAM("Replacing " << obj << " with " << inv_groups[group][0] << " due to inventory");
        }
    }
}

void ObjectListMerger::combineClassifierResults(mcr_perception_msgs::ObjectList &combined_object_list)
{
    std::vector<geometry_msgs::PoseStamped> poses;
    std::vector<std::vector<mcr_perception_msgs::Object> > pose_to_object_map;
    // create poses -> objects map
    // First add all objects from 3d list
    double height_sum = 0.0;
    for (int i = 0; i < pointcloud_object_list_.objects.size(); i++)
    {
        std::vector<mcr_perception_msgs::Object> v;
        v.push_back(pointcloud_object_list_.objects[i]);

        pose_to_object_map.push_back(v);
        poses.push_back(pointcloud_object_list_.objects[i].pose);
        height_sum += pointcloud_object_list_.objects[i].pose.pose.position.z;
    }
    // calculate average height of 3D objects
    height_sum /= pointcloud_object_list_.objects.size();

    // next add rgb poses, and if they are close to the existing ones, merge them
    for (int i = 0; i < rgb_object_list_.objects.size(); i++)
    {
        // set RGB pose to same as 3D pose
        rgb_object_list_.objects[i].pose.pose.position.z = height_sum;

        bool found_pose_in_both_lists = false;
        for (int j = 0; j < poses.size(); j++)
        {
            double distance = getDistance(poses[j].pose.position, rgb_object_list_.objects[i].pose.pose.position);
            // if both objects are close together
            if (distance < distance_threshold_)
            {
                pose_to_object_map[j].push_back(rgb_object_list_.objects[i]);
                found_pose_in_both_lists = true;
                break;
            }
        }
        if (!found_pose_in_both_lists)
        {
            std::vector<mcr_perception_msgs::Object> v;
            v.push_back(rgb_object_list_.objects[i]);
            pose_to_object_map.push_back(v);
            poses.push_back(rgb_object_list_.objects[i].pose);
        }
    }
    for (int i = 0; i < pose_to_object_map.size(); i++)
    {
        ROS_INFO_STREAM("Size of objects with this pose: " << pose_to_object_map[i].size());
    }

    std::vector<std::string> current_inventory =  objects_at_location_.objects;

    // 1st Condition
    // Both recognition matches (Not checking the Inventory)
    for (int i = 0; i < pose_to_object_map.size(); i++)
    {
        for (int j = 0; j < pose_to_object_map[i].size(); j++)
        {
            fout << pose_to_object_map[i][j].name << ", ";
        }
        fout << std::endl;
        if (pose_to_object_map[i].size() == 2)
        {
            std::string obj1 = pose_to_object_map[i][0].name;
            std::string obj2 = pose_to_object_map[i][1].name;
            std::transform(obj1.begin(), obj1.end(), obj1.begin(), ::toupper);
            std::transform(obj2.begin(), obj2.end(), obj2.begin(), ::toupper);
            if (strcasecmp(obj1.c_str(), obj2.c_str()) == 0)
            {
                combined_object_list.objects.push_back(pose_to_object_map[i][0]);
                std::vector<std::string>::iterator to_erase = std::find(current_inventory.begin(), current_inventory.end(), obj1);
                if (to_erase != current_inventory.end())
                {
                    current_inventory.erase(to_erase);
                }
            }
        }
    }

    // 2nd Condition
    // Both recognition doesnt matches (check the Inventory)
    for (int i = 0; i < pose_to_object_map.size(); i++)
    {
        if (pose_to_object_map[i].size() == 2)
        {
            std::string obj1 = pose_to_object_map[i][0].name;
            std::string obj2 = pose_to_object_map[i][1].name;
            std::transform(obj1.begin(), obj1.end(), obj1.begin(), ::toupper);
            std::transform(obj2.begin(), obj2.end(), obj2.begin(), ::toupper);
            if (strcasecmp(obj1.c_str(), obj2.c_str()) != 0)
            {
                bool object1_found = ( std::find(current_inventory.begin(), current_inventory.end(), obj1) != current_inventory.end() );
                bool object2_found = ( std::find(current_inventory.begin(), current_inventory.end(), obj2) != current_inventory.end() );

                if (object2_found and not object1_found) 
                {
                    //IF RGB found in Inventory AND 3D not in inventory THEN use RGB pose
                    combined_object_list.objects.push_back(pose_to_object_map[i][1]);
                    std::vector<std::string>::iterator to_erase = std::find(current_inventory.begin(), current_inventory.end(), obj2);
                    if (to_erase != current_inventory.end())
                    {
                        current_inventory.erase(to_erase);
                    }
                } 
                else
                {
                    //For all other condition use 3D pose
                    combined_object_list.objects.push_back(pose_to_object_map[i][0]);
                    std::vector<std::string>::iterator to_erase = std::find(current_inventory.begin(), current_inventory.end(), obj1);
                    if (to_erase != current_inventory.end())
                    {
                        current_inventory.erase(to_erase);
                    }
                }
            }
        }
    }
    // 3rd condition
    // if only one object for a given pose, only add it if it exists in the inventory
    for (int i = 0; i < pose_to_object_map.size(); i++)
    {
        if (pose_to_object_map[i].size() == 1)
        {
            std::string obj1 = pose_to_object_map[i][0].name;
            std::transform(obj1.begin(), obj1.end(), obj1.begin(), ::toupper);
            bool object1_found = ( std::find(current_inventory.begin(), current_inventory.end(), obj1) != current_inventory.end() );
            // if there's nothing in the inventory, add it
            if (objects_at_location_.objects.empty())
            {
                combined_object_list.objects.push_back(pose_to_object_map[i][0]);
            }
            else if (object1_found)
            {
                combined_object_list.objects.push_back(pose_to_object_map[i][0]);
                std::vector<std::string>::iterator to_erase = std::find(current_inventory.begin(), current_inventory.end(), obj1);
                if (to_erase != current_inventory.end())
                {
                    current_inventory.erase(to_erase);
                }
            }
        }
    }
    ROS_INFO_STREAM("Started with inventory size: " <<  objects_at_location_.objects.size() << " ended with " << current_inventory.size());
}


void ObjectListMerger::objectListCallback(const mcr_perception_msgs::ObjectList::Ptr &msg)
{
    if (accepting_object_lists_)
    {
        mergeList(msg);
    }
}

void ObjectListMerger::eventInCallback(const std_msgs::String::Ptr &msg)
{
    event_in_ = msg->data;
    event_in_received_ = true;
}

void ObjectListMerger::objectsAtLocationCallback(const mir_knowledge_base_analyzer::ObjectsAtLocation::Ptr &msg)
{
    objects_at_location_ = *msg;
}

double ObjectListMerger::getDistance(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2)
{
    return std::sqrt(std::pow(point1.x - point2.x, 2.0) +
                     std::pow(point1.y - point2.y, 2.0));
}

void ObjectListMerger::writeObjectList(const mcr_perception_msgs::ObjectList &object_list)
{
    int id = 0;
    if (object_list.objects.size() > 0)
    {
        id = object_list.objects[0].database_id;
    }
    if (id > 99)
    {
        fout << "rgb: ";
    }
    else
    {
        fout << "3D: ";
    }
    for (int i = 0; i < object_list.objects.size(); i++)
    {
        fout << object_list.objects[i].name << ", ";
    }
    fout << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_list_merger");
    ros::NodeHandle nh("~");

    int frame_rate = 5;    // in Hz
    ObjectListMerger object_list_merger;

    nh.param<int>("frame_rate", frame_rate, 5);
    ROS_INFO("node started");

    ros::Rate loop_rate(frame_rate);

    while (ros::ok())
    {
        object_list_merger.update();
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
