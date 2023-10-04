#ifndef TASK_H
#define TASK_H

#include <iostream>
#include <string>
#include <algorithm>
#include <sstream>
#include <vector>
#include "tinyxml2.h"
#include "structs.h"
#include "const.h"
#include <fstream>
#include "map.h"

class Task
{
private:
    std::map<int, Agent> agents;
public:
    bool get_task(const char* FileName, int k=-1);
    unsigned int get_agents_size() const { return agents.size(); }
    void make_ids(int width);
    void make_ij(const Map &map);
    Agent get_agent(int id) const;
    void add_agent(Agent a)
    {
        agents.insert({a.id, a});
    }
    std::vector<int> get_ids() const
    {
        std::vector<int> ids;
        for(auto a:agents)
            ids.push_back(a.first);
        return ids;
    }
    void print_task()
    {
        for(auto a:agents)
            std::cout<<"<agent start_i=\""<<a.second.start_i<<"\" start_j=\""<<a.second.start_j<<"\" goal_i=\""<<a.second.goal_i<<"\" goal_j=\""<<a.second.goal_j<<"\"/>\n";
    }
    Task();
};

#endif // TASK_H
