#include "task.h"
Task::Task()
{
    agents.clear();
}

bool Task::get_task(const char *FileName, int k)
{
    tinyxml2::XMLElement *root = 0, *agent = 0;
    tinyxml2::XMLDocument doc;

    // Load XML File
    if (doc.LoadFile(FileName) != tinyxml2::XMLError::XML_SUCCESS)
    {
        std::cout << "Error opening XML file!" << std::endl;
        return false;
    }

    // Get ROOT element
    root = doc.FirstChildElement(CNS_TAG_ROOT);
    if (!root)
    {
        std::cout << "Error! No '" << CNS_TAG_ROOT << "' tag found in XML file!" << std::endl;
        return false;
    }

    for (agent = root->FirstChildElement(); agent; agent = agent->NextSiblingElement())
    {
        Agent a;
        a.start_i = agent->DoubleAttribute(CNS_TAG_START_I);
        a.start_j = agent->DoubleAttribute(CNS_TAG_START_J);
        a.start_id = agent->IntAttribute(CNS_TAG_START_ID);
        a.goal_i = agent->DoubleAttribute(CNS_TAG_GOAL_I);
        a.goal_j = agent->DoubleAttribute(CNS_TAG_GOAL_J);
        a.goal_id = agent->IntAttribute(CNS_TAG_GOAL_ID);
        a.id = int(agents.size());
        agents[a.id] = a;
        if(int(agents.size()) == k)
            break;
    }
    return true;
}

void Task::make_ids(int width)
{
    for(size_t i = 0; i < agents.size(); i++)
    {
        agents[i].start_id = int(agents[i].start_i)*width + int(agents[i].start_j);
        agents[i].goal_id = int(agents[i].goal_i)*width + int(agents[i].goal_j);
        //std::cout<<agents[i].start_i<<" "<<agents[i].start_j<<"  "<<agents[i].goal_i<<" "<<agents[i].goal_j<<"\n";
    }
}

void Task::make_ij(const Map& map)
{
    for(unsigned int i = 0; i < agents.size(); i++)
    {
        gNode start = map.get_gNode(agents[i].start_id), goal = map.get_gNode(agents[i].goal_id);
        agents[i].start_i = start.i;
        agents[i].start_j = start.j;
        agents[i].goal_i = goal.i;
        agents[i].goal_j = goal.j;
    }

}

Agent Task::get_agent(int id) const
{
    if(agents.find(id) != agents.end())
        return agents.at(id);
    else
        return Agent();
}
