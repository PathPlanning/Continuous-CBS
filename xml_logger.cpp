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

void XML_logger::write_to_log_path(const Solution &solution)
{
    tinyxml2::XMLElement *element=doc->FirstChildElement(CNS_TAG_ROOT);
    element = element->FirstChildElement(CNS_TAG_LOG);

    tinyxml2::XMLElement *agent, *path, *hplevel;

    for(int i = 0; i < solution.paths.size(); i++)
    {
        agent = doc->NewElement(CNS_TAG_AGENT);
        agent->SetAttribute(CNS_TAG_ATTR_NUM,i);
        element->LinkEndChild(agent);

        path = doc->NewElement(CNS_TAG_PATH);
        path->SetAttribute(CNS_TAG_ATTR_NUM, i);
        path->SetAttribute(CNS_TAG_ATTR_LENGTH, solution.paths[i].cost);
        agent->LinkEndChild(path);

        int k = 0;
        hplevel = doc->NewElement(CNS_TAG_HPLEVEL);
        path->LinkEndChild(hplevel);
        k = 0;
        auto iter = solution.paths[i].nodes.begin();
        auto it = solution.paths[i].nodes.begin();
        int partnumber(0);
        tinyxml2::XMLElement *part;
        part = doc->NewElement(CNS_TAG_SECTION);
        part->SetAttribute(CNS_TAG_ATTR_NUM, partnumber);
        part->SetAttribute(CNS_TAG_ATTR_SX, it->j);
        part->SetAttribute(CNS_TAG_ATTR_SY, it->i);
        part->SetAttribute(CNS_TAG_ATTR_FX, iter->j);
        part->SetAttribute(CNS_TAG_ATTR_FY, iter->i);
        part->SetAttribute(CNS_TAG_ATTR_LENGTH, iter->g);
        hplevel->LinkEndChild(part);
        partnumber++;
        while(iter != --solution.paths[i].nodes.end())
        {
            part = doc->NewElement(CNS_TAG_SECTION);
            part->SetAttribute(CNS_TAG_ATTR_NUM, partnumber);
            part->SetAttribute(CNS_TAG_ATTR_SX, it->j);
            part->SetAttribute(CNS_TAG_ATTR_SY, it->i);
            iter++;
            part->SetAttribute(CNS_TAG_ATTR_FX, iter->j);
            part->SetAttribute(CNS_TAG_ATTR_FY, iter->i);
            part->SetAttribute(CNS_TAG_ATTR_LENGTH, iter->g - it->g);
            hplevel->LinkEndChild(part);
            it++;
            partnumber++;
        }
    }
}

void XML_logger::write_to_log_edges(const Graph &graph)
{
    tinyxml2::XMLElement *element=doc->FirstChildElement(CNS_TAG_ROOT);
    element = element->FirstChildElement(CNS_TAG_LOG);
    tinyxml2::XMLElement *edge;
    for(int i = 0; i < graph.get_nodes_size(); i++)
    {
        auto e1 = graph.get_gnode(i);
        for(auto k:e1.neighbors)
        {
            if(k < i)
                continue;
            auto e2 = graph.get_gnode(k);
            edge = doc->NewElement("edge");
            edge->SetAttribute(CNS_TAG_ATTR_SX, e1.i);
            edge->SetAttribute(CNS_TAG_ATTR_SY, e1.j);
            edge->SetAttribute(CNS_TAG_ATTR_FX, e2.i);
            edge->SetAttribute(CNS_TAG_ATTR_FY, e2.j);
            element->LinkEndChild(edge);
        }
    }

}
