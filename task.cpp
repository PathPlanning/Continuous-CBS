#include "task.h"
Task::Task()
{
    agents.clear();
}

bool Task::get_task(const char *FileName, int k)
{
    tinyxml2::XMLElement *root = 0, *agent = 0;
    tinyxml2::XMLDocument doc;
    std::string value;
    std::stringstream stream;

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
        value = agent->Value();
        std::transform(value.begin(), value.end(), value.begin(), ::tolower);
        Agent a;
        stream.clear();
        stream << agent->Attribute(CNS_TAG_START_I);
        stream >> a.start_i;
        stream.clear();
        stream << agent->Attribute(CNS_TAG_START_J);
        stream >> a.start_j;
        stream.clear();
        stream << agent->Attribute(CNS_TAG_START_ID);
        stream >> a.start_id;
        stream.clear();
        stream << agent->Attribute(CNS_TAG_GOAL_I);
        stream >> a.goal_i;
        stream.clear();
        stream << agent->Attribute(CNS_TAG_GOAL_J);
        stream >> a.goal_j;
        stream.clear();
        stream << agent->Attribute(CNS_TAG_GOAL_ID);
        stream >> a.goal_id;
        a.id = agents.size();
        agents.push_back(a);
        if(agents.size() == k)
            break;
    }
}

void Task::make_ids(int width)
{
    for(size_t i = 0; i < agents.size(); i++)
    {
        agents[i].start_id = agents[i].start_i*width + agents[i].start_j;
        agents[i].goal_id = agents[i].goal_i*width + agents[i].goal_j;
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
    if(id >= 0 && id < agents.size())
        return agents[id];
    else
        return Agent();
}
