/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#include <iostream>
#include <mir_planner_executor/actions/mockup_helper.h>

bool mockupAsk(std::string action, std::vector<std::string> args) {
    std::cout << std::endl << "==============================================" << std::endl;
    std::cout << "Mockup: \"" << action << "\"";
    if (!args.empty()) {
        std::cout << ", args:" << std::endl;
        std::cout << "    ";
        for(auto& arg: args) {
            std::cout << arg << ", ";
        }
        std::cout << std::endl;
    } else {
        std::cout << ": " << std::endl;
    }
    std::cout << "Succeed? (y|n)[y]? ";
    std::string ok;
    getline(std::cin, ok);
    if (ok.empty() || ok == "y") {
        return true;
    } else {
        return false;
    }
};
