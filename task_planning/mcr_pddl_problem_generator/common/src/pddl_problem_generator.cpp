/*
 * Copyright [2016] <Bonn-Rhein-Sieg University>
 *
 * Author: Oscar Lima (olima_84@yahoo.com)
 * Based on: https://github.com/KCL-Planning/ROSPlan/blob/master/rosplan_planning_system/src/generatePDDLProblem.cpp
 *
 * Generic class used to generate a PDDL problem file with or without cost information
 *
 */

#include <mcr_pddl_problem_generator/pddl_problem_generator.h>
#include <map>
#include <string>
#include <vector>
#include <algorithm>
#include <utility>
#include <boost/filesystem.hpp>

PDDLProbGenCost::PDDLProbGenCost(std::string& problem_path, std::string& metric)
: problem_path_(problem_path), metric_(metric), ready_to_generate_(true) {

}

void PDDLProbGenCost::addProbabilityObject(std::string str, float f) {
    probabilities_objects_[str] = f;
    for(int i = 0;i<100;i++) {
        std::ostringstream stringStream;
        stringStream << str << "-" << std::setfill('0') << std::setw(2) << i;
        std::string id = stringStream.str();
        probabilities_objects_[id] = f;
    }
}

bool PDDLProbGenCost::generatePDDLProblemFile(KCL_rosplan::PlanningEnvironment& environment)
{

    probabilities_actions_["at"] = 0.0f;
    probabilities_actions_["on"] = 1.0f;
    probabilities_actions_["insert"] = 0.5f;

    probabilities_locations_["sh01"] = 0.1f;
    probabilities_locations_["sh02"] = 0.1f;

    probabilities_locations_["ws05"] = 0.2f;
    probabilities_locations_["ws07"] = 0.2f;
    probabilities_locations_["ws08"] = 0.2f;

    probabilities_locations_["pp01_cavity"] = 0.5f;

    addProbabilityObject("bearing_box", 0.5);
    addProbabilityObject("r20", 0.5);
    addProbabilityObject("motor", 0.5);
    addProbabilityObject("m30", 0.5);
    addProbabilityObject("axis", 0.2);
    addProbabilityObject("distance_tube", 0.2);
    addProbabilityObject("bearing", 0.2);
    addProbabilityObject("m20", 0.2);
    addProbabilityObject("container_box_blue", 0.2);
    addProbabilityObject("container_box_red", 0.2);
    addProbabilityObject("m20_100", 0.0);

    // check if configure method was called, if not then exit without doing anything
    if (!ready_to_generate_) return false;

    boost::filesystem::path boost_problem_file((problem_path_).c_str());
    boost::filesystem::path boost_problem_dir = boost_problem_file.parent_path();

    if(!(boost::filesystem::exists(boost_problem_dir))){
        if (!(boost::filesystem::create_directory(boost_problem_dir)))
            return false;
    }

    std::ofstream pFile;
    pFile.open((problem_path_).c_str());

    if (!makeHeader(environment, pFile))
    {
        std::cerr << "Error : Could not make header" << std::endl;
        pFile.close();
        return false;
    }

    if (!makeObjects(environment, pFile))
    {
        std::cerr << "Error : Could not make objects" << std::endl;
        pFile.close();
        return false;
    }

    if (!makeInitialStateHeader(pFile))
    {
        std::cerr << "Error : Could not make initial state header" << std::endl;
        pFile.close();
        return false;
    }

    if (!makeInitialStateCost(environment, pFile))
    {
        std::cerr << "Error : Could not make initial state costs" << std::endl;
        pFile.close();
        return false;
    }

    if (!makeInitialStateFacts(environment, pFile))
    {
        std::cerr << "Error : Could not make state facts" << std::endl;
        pFile.close();
        return false;
    }

    if (!makeGoals(environment, pFile))
    {
        std::cerr << "Error : Could not make goals" << std::endl;
        pFile.close();
        return false;
    }

    if (!makeMetric(pFile))
    {
        std::cerr << "Error : Could not make metric" << std::endl;
        pFile.close();
        return false;
    }

    if (!finalizePDDLFile(pFile))
    {
        std::cerr << "Error : Could not finalize PDDL file" << std::endl;
        pFile.close();
        return false;
    }

    pFile.close();
    return true;
}

bool PDDLProbGenCost::makeHeader(KCL_rosplan::PlanningEnvironment& environment, std::ofstream &pFile)
{
    // check if configure method was called, if not then exit without doing anything
    if (!ready_to_generate_) return false;

    pFile << ";This PDDL problem definition was made automatically from a KB snapshot" << std::endl;
    pFile << "(define (problem " << environment.domainName << "_task)" << std::endl;
    pFile << "(:domain " << environment.domainName << ")" << std::endl;
    pFile << "" << std::endl;

    return true;
}

bool PDDLProbGenCost::makeObjects(KCL_rosplan::PlanningEnvironment& environment, std::ofstream &pFile)
{
    // check if configure method was called, if not then exit without doing anything
    if (!ready_to_generate_) return false;

    bool is_there_objects = false;

    // objects
    pFile << "(:objects" << std::endl;

    for (std::map<std::string, std::vector<std::string> >::iterator iit=environment.type_object_map.begin();
         iit != environment.type_object_map.end(); ++iit)
    {
        if (iit->second.size() > 0)
        {
            pFile << "    ";

            for (size_t i = 0; i < iit->second.size(); i++)
            {
                pFile << iit->second[i] << " ";
            }

            pFile << "- " << iit->first << std::endl;
            if (!is_there_objects) is_there_objects = true;
        }
    }
    pFile << ")" << std::endl;
    pFile << "" << std::endl;

    return is_there_objects;
}

bool PDDLProbGenCost::makeInitialStateHeader(std::ofstream &pFile)
{
    // check if configure method was called, if not then exit without doing anything
    if (!ready_to_generate_) return false;

    pFile << "(:init" << std::endl;

    return true;
}

bool PDDLProbGenCost::makeInitialStateCost(KCL_rosplan::PlanningEnvironment& environment, std::ofstream &pFile)
{
    // check if configure method was called, if not then exit without doing anything
    if (!ready_to_generate_) return false;

    pFile << "    ;Cost information starts" << std::endl;
    pFile << "    (= (total-cost) 0)" << std::endl;
    pFile << "    ;Cost information ends" << std::endl;
    pFile << "" << std::endl;

    return true;
}

bool PDDLProbGenCost::makeInitialStateFacts(KCL_rosplan::PlanningEnvironment& environment, std::ofstream &pFile)
{
    // check if configure method was called, if not then exit without doing anything
    if (!ready_to_generate_) return false;

    bool is_there_facts = false;

    // add knowledge to the initial state
    for (size_t i = 0; i < environment.domain_attributes.size(); i++)
    {
        std::stringstream ss;
        ss << "    (";

        if (environment.domain_attributes[i].knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::FUNCTION)
        {
            ss << "= (";
        }

        ss << environment.domain_attributes[i].attribute_name;

        // fetch the corresponding symbols from domain
        std::map<std::string, std::vector<std::string> >::iterator ait;
        ait = environment.domain_predicates.find(environment.domain_attributes[i].attribute_name);

        if (ait == environment.domain_predicates.end())
        {
            ait = environment.domain_functions.find(environment.domain_attributes[i].attribute_name);
        }

        if (ait == environment.domain_functions.end())
        {
            continue;
        }

        // find the PDDL parameters in the KnowledgeItem
        bool writeAttribute = true;

        for (size_t j=0; j < ait->second.size(); j++)
        {
            bool found = false;
            for (size_t k = 0; k < environment.domain_attributes[i].values.size(); k++)
            {
                if (0 == environment.domain_attributes[i].values[k].key.compare(ait->second[j]))
                {
                    ss << " " << environment.domain_attributes[i].values[k].value;
                    found = true;
                }
            }
            if (!found) writeAttribute = false;
        }

        ss << ")";

        if (!is_there_facts) is_there_facts = true;

        // output function value
        if (environment.domain_attributes[i].knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::FUNCTION)
        {
            ss << " " << environment.domain_attributes[i].function_value << ")";
        }

        if (writeAttribute) pFile << ss.str() << std::endl;
    }

    // add knowledge to the initial state
    for (size_t i = 0; i < environment.instance_attributes.size(); i++)
    {
        std::stringstream ss;
        bool writeAttribute = false;

        // check if attribute is a PDDL predicate
        std::map<std::string, std::vector<std::string> >::iterator ait;

        ait = environment.domain_predicates.find(environment.instance_attributes[i].attribute_name);

        if (ait != environment.domain_predicates.end())
        {
            writeAttribute = true;

            ss << "    (" + environment.instance_attributes[i].attribute_name;

            // find the PDDL parameters in the KnowledgeItem
            for (size_t j = 0; j < ait->second.size(); j++)
            {
                bool found = false;

                for (size_t k = 0; k < environment.instance_attributes[i].values.size(); k++)
                {
                    if (0 == environment.instance_attributes[i].values[k].key.compare(ait->second[j]))
                    {
                        ss << " " << environment.instance_attributes[i].values[k].value;
                        found = true;
                    }
                }
                if (!found) writeAttribute = false;
            };
            ss << ")";
        }
        if (writeAttribute) pFile << ss.str() << std::endl;
    }
    pFile << ")" << std::endl;

    // blank space between facts and goals
    pFile << "" << std::endl;

    return is_there_facts;
}

float PDDLProbGenCost::getProbability(std::string str, std::map<std::string, float>& probabilities) {
    std::transform(str.begin(), str.end(), str.begin(), ::tolower);
    std::map<std::string, float>::iterator it = probabilities.find(str);
    if(it == probabilities.end()) {
        return 1.0f;
    } else {
        return it->second;
    }
}
float PDDLProbGenCost::getProbabilityAction(std::string str) {
    return getProbability(str, probabilities_actions_);
}
float PDDLProbGenCost::getProbabilityObject(std::string str) {
    return getProbability(str, probabilities_objects_);
}
float PDDLProbGenCost::getProbabilityLocation(std::string str) {
    return getProbability(str, probabilities_locations_);
}

bool PDDLProbGenCost::makeGoals(KCL_rosplan::PlanningEnvironment& environment, std::ofstream &pFile)
{
    // check if configure method was called, if not then exit without doing anything
    if (!ready_to_generate_) return false;

    std::vector<std::pair<float, std::string> > goals;

    bool is_there_goals = false;

    pFile << "(:goal (and" << std::endl;

    // propositions in the initial state
    for (size_t i = 0; i < environment.goal_attributes.size(); i++)
    {
        std::stringstream ss;
        bool writeAttribute = true;

        std::string action_name = environment.goal_attributes[i].attribute_name;
        float probability = getProbabilityAction(action_name);

        // check if attribute belongs in the PDDL model
        std::map<std::string, std::vector<std::string> >::iterator ait;
        ait = environment.domain_predicates.find(environment.goal_attributes[i].attribute_name);
        if (ait != environment.domain_predicates.end())
        {
            ss << "    (" + environment.goal_attributes[i].attribute_name;

            // find the PDDL parameters in the KnowledgeItem
            bool found = false;
            for (size_t j = 0; j < ait->second.size(); j++) {
                for (size_t k = 0; k < environment.goal_attributes[i].values.size(); k++)
                {
                    if (0 == environment.goal_attributes[i].values[k].key.compare(ait->second[j]))
                    {
                        std::string name = environment.goal_attributes[i].values[k].value;
                        ss << " " << environment.goal_attributes[i].values[k].value;
                        found = true;

                        if(action_name.compare("on") == 0) {
                            if(k == 0) {
                                probability *= getProbabilityObject(name);
                            } else if(k == 1) {
                                probability *= getProbabilityLocation(name);
                            }
                        } else if(action_name.compare("in") == 0) {
                            probability *= getProbabilityObject(name);
                        }
                    }
                }
            }
            if (!found) writeAttribute = false;

            if (!is_there_goals) is_there_goals = true;

            ss << ")";
        } else {
            writeAttribute = false;
        }
        if (writeAttribute) {
            goals.push_back(std::make_pair (probability,ss.str()));
        }
    }

    //bool wayToSort(std::pair<float, std::string> a, std::pair<float, std::string> b) { return i > j; }
    std::sort(goals.begin(), goals.end(), goal_sort_());

    uint count = 0;
    for(std::vector<std::pair<float, std::string> >::iterator it = goals.begin(); it != goals.end(); ++it) {
        if(count >= (max_goals_)) {
            break;
        }
        std::cout << (*it).second << " " << (*it).first << std::endl;
        pFile << (*it).second << std::endl;
        count++;
    }

    pFile << "    )" << std::endl;
    pFile << ")" << std::endl;
    pFile << "" << std::endl;

    return is_there_goals;
}

bool PDDLProbGenCost::makeMetric(std::ofstream& pFile)
{
    // check if configure method was called, if not then exit without doing anything
    if (!ready_to_generate_) return false;

    // metric specification
    pFile << metric_ << std::endl;
    pFile << "" << std::endl;

    return true;
}

bool PDDLProbGenCost::finalizePDDLFile(std::ofstream& pFile)
{
    // check if configure method was called, if not then exit without doing anything
    if (!ready_to_generate_) return false;

    // end of problem
    pFile << ")" << std::endl;

    return true;
}

void PDDLProbGenCost::setMaxGoals(int max_goals)
{
    max_goals_ = max_goals;
}

