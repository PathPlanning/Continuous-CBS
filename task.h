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
    std::vector<Agent> agents;
public:
    bool get_task(const char* FileName, int k=-1);
    unsigned int get_agents_size() const { return agents.size(); }
    void make_ids(int width);
    void make_ij(const Map &map);
    Agent get_agent(int id) const;
    void add_agent(Agent a)
    {
        a.id = agents.size();
        agents.push_back(a);
    }
    void print_task()
    {
        //for(int i=0; i<agents.size(); i++)
        //    std::cout<<i<<","<<agents[i].start_i<<","<<agents[i].start_j<<","<<agents[i].goal_i<<","<<agents[i].goal_j<<"\n";
        for(auto agent:agents)
            std::cout<<"<agent start_i=\""<<agent.start_i<<"\" start_j=\""<<agent.start_j<<"\" goal_i=\""<<agent.goal_i<<"\" goal_j=\""<<agent.goal_j<<"\"/>\n";
    }
    Task();
};

#endif // TASK_H
