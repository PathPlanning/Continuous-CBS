#include "xml_logger.h"

XML_logger::XML_logger()
{

}
void XML_logger::save_log()
{
    doc->SaveFile(LogFileName.c_str());
}

bool XML_logger::get_log(const char *FileName)
{
    std::string value;
    tinyxml2::XMLDocument doc_xml;

    if(doc_xml.LoadFile(FileName) != tinyxml2::XMLError::XML_SUCCESS)
    {
        std::cout << "Error opening XML-file in getLog";
        return false;
    }

    value = FileName;
    size_t dotPos = value.find_last_of(".");

    if(dotPos != std::string::npos)
        value.insert(dotPos,CN_LOG);
    else
        value += CN_LOG;

    LogFileName = value;
    doc_xml.SaveFile(LogFileName.c_str());

    doc = new tinyxml2::XMLDocument;
    doc->LoadFile(LogFileName.c_str());

    /*tinyxml2::XMLElement *root = doc->FirstChildElement(CNS_TAG_ROOT);
    tinyxml2::XMLElement *log = doc->NewElement(CNS_TAG_LOG);
    root->LinkEndChild(log);
    tinyxml2::XMLElement *sum = doc->NewElement(CNS_TAG_SUM);
    log->LinkEndChild(sum);
    tinyxml2::XMLElement* path = doc->NewElement(CNS_TAG_PATH);
    log->LinkEndChild(path);*/

    return true;
}

void XML_logger::write_to_log_summary(const Solution &solution)
{
    tinyxml2::XMLElement *root = doc->FirstChildElement(CNS_TAG_ROOT);
    tinyxml2::XMLElement *log = doc->NewElement(CNS_TAG_LOG);
    root->LinkEndChild(log);
    tinyxml2::XMLElement *sum = doc->NewElement(CNS_TAG_SUM);
    log->LinkEndChild(sum);
    tinyxml2::XMLElement *element = log->FirstChildElement(CNS_TAG_SUM);

    element->SetAttribute(CNS_TAG_ATTR_TIME, solution.time.count());
    element->SetAttribute(CNS_TAG_ATTR_FLOWTIME, solution.flowtime);
    element->SetAttribute(CNS_TAG_ATTR_MAKESPAN, solution.makespan);
}

void XML_logger::write_to_log_path(const Solution &solution, const Map &map)
{
    tinyxml2::XMLElement *element=doc->FirstChildElement(CNS_TAG_ROOT);
    element = element->FirstChildElement(CNS_TAG_LOG);

    tinyxml2::XMLElement *agent, *path;

    for(int i = 0; i < int(solution.paths.size()); i++)
    {
        agent = doc->NewElement(CNS_TAG_AGENT);
        agent->SetAttribute(CNS_TAG_ATTR_NUM,i);
        element->LinkEndChild(agent);

        path = doc->NewElement(CNS_TAG_PATH);
        //path->SetAttribute(CNS_TAG_ATTR_NUM, i);
        path->SetAttribute(CNS_TAG_ATTR_DURATION, solution.paths[i].cost);
        agent->LinkEndChild(path);

        //hplevel = doc->NewElement(CNS_TAG_HPLEVEL);
        //path->LinkEndChild(hplevel);
        auto iter = solution.paths[i].nodes.begin();
        auto it = solution.paths[i].nodes.begin();
        int partnumber(0);
        tinyxml2::XMLElement *part;
        /*part = doc->NewElement(CNS_TAG_SECTION);
        part->SetAttribute(CNS_TAG_ATTR_NUM, partnumber);
        part->SetAttribute(CNS_TAG_START_I, map.get_i(it->id));
        part->SetAttribute(CNS_TAG_START_J, map.get_j(it->id));
        part->SetAttribute(CNS_TAG_GOAL_I, map.get_i(iter->id));
        part->SetAttribute(CNS_TAG_GOAL_J, map.get_j(iter->id));
        part->SetAttribute(CNS_TAG_ATTR_LENGTH, iter->g);
        hplevel->LinkEndChild(part);
        partnumber++;*/
        while(iter != std::prev(solution.paths[i].nodes.end()))
        {
            part = doc->NewElement(CNS_TAG_SECTION);
            part->SetAttribute(CNS_TAG_ATTR_NUM, partnumber);
            part->SetAttribute(CNS_TAG_START_I, map.get_i(it->id));
            part->SetAttribute(CNS_TAG_START_J, map.get_j(it->id));
            iter++;
            part->SetAttribute(CNS_TAG_GOAL_I, map.get_i(iter->id));
            part->SetAttribute(CNS_TAG_GOAL_J, map.get_j(iter->id));
            part->SetAttribute(CNS_TAG_ATTR_DURATION, iter->g - it->g);
            path->LinkEndChild(part);
            it++;
            partnumber++;
        }
    }
}
