#include "task.h"
using namespace tinyxml2;
Task::Task()
{
    agents.clear();
}

bool Task::get_task(const char *FileName, int k)
{
    XMLDocument doc;
    if(doc.LoadFile(FileName) != XMLError::XML_SUCCESS)
    {
        std::cout << "Error openning input XML file."<<std::endl;
        return false;
    }
    XMLElement *root = doc.FirstChildElement(CNS_TAG_ROOT);
    if (!root)
    {
        std::cout << "No '"<<CNS_TAG_ROOT<<"' element found in XML task-file."<<std::endl;
        return false;
    }

    double defaultSize(CN_DEFAULT_SIZE), defaultRSpeed(CN_DEFAULT_RSPEED), defaultMSpeed(CN_DEFAULT_MSPEED),
           defaultSHeading(CN_DEFAULT_SHEADING), defaultGHeading(CN_DEFAULT_GHEADING);

    root = root->FirstChildElement(CNS_TAG_AGENTS);
    if (!root)
    {
        std::cout << "No '"<<CNS_TAG_AGENTS<<"' element found in XML file."<<std::endl;
        return false;
    }
    XMLElement *element = root->FirstChildElement(CNS_TAG_DEF_PARAMS);
    if(element)
    {
        defaultSize = element->DoubleAttribute(CNS_TAG_ATTR_SIZE);
        defaultRSpeed = element->DoubleAttribute(CNS_TAG_ATTR_RSPEED);
        defaultMSpeed = element->DoubleAttribute(CNS_TAG_ATTR_MSPEED);
        defaultSHeading = element->DoubleAttribute(CNS_TAG_ATTR_SHEADING);
        defaultGHeading = element->DoubleAttribute(CNS_TAG_ATTR_GHEADING);
        if(element->Attribute(CNS_TAG_ATTR_GHEADING) == CNS_HEADING_WHATEVER)
            defaultGHeading = CN_HEADING_WHATEVER;
        if(defaultSize <= 0 || defaultSize > 0.5)
        {
            std::cout<<"Incorrect value of default "<<CNS_TAG_ATTR_SIZE" parameter. The value is set to "<< CN_DEFAULT_SIZE<<".\n";
            defaultSize = CN_DEFAULT_SIZE;
        }
        if(defaultRSpeed <= 0 || defaultRSpeed > 10.0)
        {
            std::cout<<"Incorrect value of default "<<CNS_TAG_ATTR_RSPEED<<" parameter. The value is set to "<< CN_DEFAULT_RSPEED<<".\n";
            defaultRSpeed = CN_DEFAULT_RSPEED;
        }
        if(defaultMSpeed <= 0 || defaultMSpeed > 10.0)
        {
            std::cout<<"Incorrect value of default "<<CNS_TAG_ATTR_MSPEED<<" parameter. The value is set to "<< CN_DEFAULT_MSPEED<<".\n";
            defaultMSpeed = CN_DEFAULT_MSPEED;
        }
        if(defaultSHeading < 0 || defaultSHeading > 360)
        {
            std::cout<<"Incorrect value of default "<<CNS_TAG_ATTR_SHEADING<<" parameter. The value is set to "<< CN_DEFAULT_SHEADING<<".\n";
            defaultSHeading = CN_DEFAULT_SHEADING;
        }
        if(defaultGHeading < -1 || defaultGHeading > 360)
        {
            std::cout<<"Incorrect value of default "<<CNS_TAG_ATTR_GHEADING<<" parameter. The value is set to "<< CN_DEFAULT_GHEADING<<".\n";
            defaultGHeading = CN_DEFAULT_GHEADING;
        }
    }
    element = root->FirstChildElement(CNS_TAG_AGENT);
    if(!element)
    {
        std::cout << "No '"<<CNS_TAG_AGENT<<"' element found in XML file."<<std::endl;
        return false;
    }
    int id(0);
    for(element; element; element = element->NextSiblingElement("agent"))
    {
        Agent agent;
        agent.start_i = element->IntAttribute(CNS_TAG_ATTR_SY);
        agent.start_j = element->IntAttribute(CNS_TAG_ATTR_SX);
        agent.goal_i = element->IntAttribute(CNS_TAG_ATTR_GY);
        agent.goal_j = element->IntAttribute(CNS_TAG_ATTR_GX);
        agent.start_id = element->IntAttribute("start_id");
        agent.goal_id = element->IntAttribute("goal_id");
        if(element->Attribute(CNS_TAG_ATTR_ID))
            agent.task_id = element->Attribute(CNS_TAG_ATTR_ID);
        else
            agent.task_id = std::to_string(id);
        agent.id = id;
        id++;
        if(element->Attribute(CNS_TAG_ATTR_SIZE))
            agent.size = element->DoubleAttribute(CNS_TAG_ATTR_SIZE);
        else
            agent.size = defaultSize;
        if(element->Attribute(CNS_TAG_ATTR_RSPEED))
            agent.rspeed = element->DoubleAttribute(CNS_TAG_ATTR_RSPEED);
        else
            agent.rspeed = defaultRSpeed;
        if(element->Attribute(CNS_TAG_ATTR_MSPEED))
            agent.mspeed = element->DoubleAttribute(CNS_TAG_ATTR_MSPEED);
        else
            agent.mspeed = defaultMSpeed;
        if(element->Attribute(CNS_TAG_ATTR_SHEADING))
            agent.start_heading = element->DoubleAttribute(CNS_TAG_ATTR_SHEADING);
        else
            agent.start_heading = defaultSHeading;
        if(element->Attribute(CNS_TAG_ATTR_GHEADING))
            agent.goal_heading = element->DoubleAttribute(CNS_TAG_ATTR_GHEADING);
        else
            agent.goal_heading = defaultGHeading;
        if(agent.size <= 0 || agent.size > 0.5)
        {
            std::cout<<"Incorrect value of "<<CNS_TAG_ATTR_SIZE<<" attribute of agent "<<agent.id<<". It's set to default value "<<defaultSize<<".\n";
            agent.size = defaultSize;
        }
        if(agent.rspeed <= 0 || agent.rspeed > 10.0)
        {
            std::cout<<"Incorrect value of "<<CNS_TAG_ATTR_RSPEED<<" of agent "<<agent.id<<". It's set to default value "<<defaultRSpeed<<".\n";
            agent.rspeed = defaultRSpeed;
        }
        if(agent.mspeed <= 0 || agent.mspeed > 10.0)
        {
            std::cout<<"Incorrect value of "<<CNS_TAG_ATTR_MSPEED<<" of agent "<<agent.id<<". It's set to default value "<<defaultMSpeed<<".\n";
            agent.mspeed = defaultMSpeed;
        }
        if(agent.start_heading < 0 || agent.start_heading > 360.0)
        {
            std::cout<<"Incorrect value of "<<CNS_TAG_ATTR_SHEADING<<" of agent "<<agent.id<<". It's set to default value "<<defaultSHeading<<".\n";
            agent.start_heading = defaultSHeading;
        }
        if(agent.goal_heading < -1 || agent.goal_heading > 360.0)
        {
            std::cout<<"Incorrect value of "<<CNS_TAG_ATTR_GHEADING<<" of agent "<<agent.id<<". It's set to default value "<<defaultSHeading<<".\n";
            agent.goal_heading = defaultGHeading;
        }
        agent.mspeed = 1.0;
        agent.rspeed = 0.25;
        agents.push_back(agent);
        if(agents.size() == k)
            break;
    }
    return true;
}

void Task::make_ids(int width)
{
    for(size_t i = 0; i < agents.size(); i++)
    {
        agents[i].start_id = agents[i].start_i*width + agents[i].start_j;
        agents[i].goal_id = agents[i].goal_i*width + agents[i].goal_j;
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
