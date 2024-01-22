#include "config.h"
using namespace  tinyxml2;
Config::Config()
{
    connectdness = CN_CONNECTEDNESS;
    use_cardinal = CN_USE_CARDINAL;
    agent_size = CN_AGENT_SIZE;
    timelimit = CN_TIMELIMIT;
    focal_weight = CN_FOCAL_WEIGHT;
    precision = CN_PRECISION;
    hlh_type = CN_HLH_TYPE;
    use_disjoint_splitting = false;
    mc_type = 0;
}


void Config::getConfig(const char *fileName)
{
    std::stringstream stream;
    XMLDocument doc;
    if (doc.LoadFile(fileName) != tinyxml2::XMLError::XML_SUCCESS)
    {
        std::cout << "Error opening Config XML file!" << std::endl;
        return;
    }

    XMLElement *root = doc.FirstChildElement(CNS_TAG_ROOT);
    if (!root)
    {
        std::cout << "No 'root' element found in XML file."<<std::endl;
        return;
    }

    XMLElement *algorithm = root->FirstChildElement(CNS_TAG_ALGORITHM);
    if(!algorithm)
    {
        std::cout << "No 'algorithm' element found in XML file."<<std::endl;
        return;
    }

    XMLElement *element = algorithm->FirstChildElement("precision");
    if (!element)
    {
        std::cout << "Error! No 'precision' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to '"<<CN_PRECISION<<"'."<<std::endl;
        precision = CN_PRECISION;
    }
    else
    {
        auto value = element->GetText();
        stream<<value;
        stream>>precision;
        if(precision > 1.0 || precision <= 0)
        {
            std::cout << "Error! Wrong 'precision' value found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to '"<<CN_PRECISION<<"'."<<std::endl;
            precision = CN_PRECISION;
        }
        stream.clear();
        stream.str("");
    }

    element = algorithm->FirstChildElement("use_cardinal");
    if (!element)
    {
        std::cout << "Error! No 'use_cardinal' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to '"<<CN_USE_CARDINAL<<"'."<<std::endl;
        use_cardinal = CN_USE_CARDINAL;
    }
    else
    {
        std::string value = element->GetText();
        if(value.compare("true") == 0 || value.compare("1") == 0)
        {
            use_cardinal = true;
        }
        else if(value.compare("false") == 0 || value.compare("0") == 0)
        {
            use_cardinal = false;
        }
        else
        {
            std::cout << "Error! Wrong 'use_cardinal' value found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to '"<<CN_USE_CARDINAL<<"'."<<std::endl;
            use_cardinal = CN_USE_CARDINAL;
        }
    }

    element = algorithm->FirstChildElement("use_disjoint_splitting");
    if (!element)
    {
        std::cout << "Error! No 'use_disjoint_splitting' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to '"<<CN_USE_DS<<"'."<<std::endl;
        use_disjoint_splitting = CN_USE_DS;
    }
    else
    {
        std::string value = element->GetText();
        if(value.compare("true") == 0 || value.compare("1") == 0)
        {
            use_disjoint_splitting = true;
        }
        else if(value.compare("false") == 0 || value.compare("0") == 0)
        {
            use_disjoint_splitting = false;
        }
        else
        {
            std::cout << "Error! Wrong 'use_disjoint_splitting' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to '"<<CN_USE_DS<<"'."<<std::endl;
            use_disjoint_splitting = CN_USE_DS;
        }
    }

    element = algorithm->FirstChildElement("mc_type");
    if (!element)
    {
        std::cout << "Error! No 'mc_type' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to '"<<0<<"'."<<std::endl;
        mc_type = 0;
    }
    else
    {
        std::string value = element->GetText();

        stream<<value;
        stream>>mc_type;
        stream.clear();
        stream.str("");
    }

    element = algorithm->FirstChildElement("connectedness");
    if (!element)
    {
        std::cout << "Error! No 'connectedness' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to '"<<CN_CONNECTEDNESS<<"'."<<std::endl;
        connectdness = CN_CONNECTEDNESS;
    }
    else
    {
        auto value = element->GetText();
        stream<<value;
        stream>>connectdness;
        stream.clear();
        stream.str("");
    }

    element = algorithm->FirstChildElement("focal_weight");
    if (!element)
    {
        std::cout << "Error! No 'focal_weight' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to '"<<CN_FOCAL_WEIGHT<<"'."<<std::endl;
        focal_weight = CN_FOCAL_WEIGHT;
    }
    else
    {
        auto value = element->GetText();
        stream<<value;
        stream>>focal_weight;
        if(focal_weight < 1.0)
        {
            std::cout << "Error! Wrong 'focal_weight' value found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to '"<<CN_FOCAL_WEIGHT<<"'."<<std::endl;
            focal_weight = CN_FOCAL_WEIGHT;
        }
        stream.clear();
        stream.str("");
    }

    element = algorithm->FirstChildElement("agent_size");
    if (!element)
    {
        std::cout << "Error! No 'agent_size' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to '"<<CN_AGENT_SIZE<<"'."<<std::endl;
        agent_size = CN_AGENT_SIZE;
    }
    else
    {
        auto value = element->GetText();
        stream<<value;
        stream>>agent_size;
        if(agent_size < 0 || agent_size > 0.5)
        {
            std::cout << "Error! Wrong 'agent_size' value found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to '"<<CN_AGENT_SIZE<<"'."<<std::endl;
            agent_size = CN_AGENT_SIZE;
        }
        stream.clear();
        stream.str("");
    }

    element = algorithm->FirstChildElement("hlh_type");
    if (!element)
    {
        std::cout << "Error! No 'hlh_type' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to '"<<CN_HLH_TYPE<<"'."<<std::endl;
        hlh_type = CN_HLH_TYPE;
    }
    else
    {
        auto value = element->GetText();
        stream<<value;
        stream>>hlh_type;
        if(hlh_type < 0 || hlh_type > 2)
        {
            std::cout << "Error! Wrong 'hlh_type' value found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to '"<<CN_HLH_TYPE<<"'."<<std::endl;
            hlh_type = CN_HLH_TYPE;
        }
        stream.clear();
        stream.str("");
    }

    element = algorithm->FirstChildElement("timelimit");
    if (!element)
    {
        std::cout << "Error! No 'timelimit' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to '"<<CN_TIMELIMIT<<"'."<<std::endl;
        timelimit = CN_TIMELIMIT;
    }
    else
    {
        auto value = element->GetText();
        stream<<value;
        stream>>timelimit;
        if(timelimit <= 0)
            timelimit = CN_INFINITY;
        stream.clear();
        stream.str("");
    }
    return;
}
