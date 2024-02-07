#include "config.h"
using namespace  tinyxml2;
Config::Config()
{
    agent_size = CN_AGENT_SIZE;
    timelimit = CN_TIMELIMIT;
    focal_weight = CN_FOCAL_WEIGHT;
    precision = CN_PRECISION;
    use_disjoint_splitting = CN_USE_DS;
    mc_type = CN_MC_TYPE;
}


void Config::getConfig(const char *fileName)
{
    std::stringstream stream;
    XMLDocument doc;
    if (doc.LoadFile(fileName) != tinyxml2::XMLError::XML_SUCCESS)
    {
        std::cout << "Warning! Cannot open Config XML file! All the settings are set to default values!" << std::endl;
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

    XMLElement *element = algorithm->FirstChildElement(CNS_TAG_PRECISON);
    if (!element)
    {
        std::cout << "Error! No '"<<CNS_TAG_PRECISON<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to '"<<CN_PRECISION<<"'."<<std::endl;
        precision = CN_PRECISION;
    }
    else
    {
        auto value = element->GetText();
        stream<<value;
        stream>>precision;
        if(precision >= 1.0 || precision <= 0)
        {
            std::cout << "Error! Wrong '"<<CNS_TAG_PRECISON<<"' value found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to '"<<CN_PRECISION<<"'."<<std::endl;
            precision = CN_PRECISION;
        }
        stream.clear();
        stream.str("");
    }

    element = algorithm->FirstChildElement(CNS_TAG_USE_DS);
    if (!element)
    {
        std::cout << "Error! No '"<<CNS_TAG_USE_DS<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to '"<<CN_USE_DS<<"'."<<std::endl;
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
            std::cout << "Error! Wrong '"<<CNS_TAG_USE_DS<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to '"<<CN_USE_DS<<"'."<<std::endl;
            use_disjoint_splitting = CN_USE_DS;
        }
    }

    element = algorithm->FirstChildElement(CNS_TAG_MCTYPE);
    if (!element)
    {
        std::cout << "Error! No '"<<CNS_TAG_MCTYPE<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to '"<<0<<"'."<<std::endl;
        mc_type = 0;
    }
    else
    {
        std::string value = element->GetText();

        stream<<value;
        stream>>mc_type;
        stream.clear();
        stream.str("");
        if(0 > mc_type || mc_type > 3)
        {
            std::cout << "Error! Wrong '"<<CNS_TAG_MCTYPE<<"' value found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to '"<<CN_MC_TYPE<<"'."<<std::endl;
            mc_type = CN_MC_TYPE;
        }
    }

    element = algorithm->FirstChildElement(CNS_TAG_FOCALW);
    if (!element)
    {
        std::cout << "Error! No '"<<CNS_TAG_FOCALW<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to '"<<CN_FOCAL_WEIGHT<<"'."<<std::endl;
        focal_weight = CN_FOCAL_WEIGHT;
    }
    else
    {
        auto value = element->GetText();
        stream<<value;
        stream>>focal_weight;
        if(focal_weight < 1.0)
        {
            std::cout << "Error! Wrong '"<<CNS_TAG_FOCALW<<"' value found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to '"<<CN_FOCAL_WEIGHT<<"'."<<std::endl;
            focal_weight = CN_FOCAL_WEIGHT;
        }
        stream.clear();
        stream.str("");
    }

    element = algorithm->FirstChildElement(CNS_TAG_AGENTSIZE);
    if (!element)
    {
        std::cout << "Error! No '"<<CNS_TAG_AGENTSIZE<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to '"<<CN_AGENT_SIZE<<"'."<<std::endl;
        agent_size = CN_AGENT_SIZE;
    }
    else
    {
        auto value = element->GetText();
        stream<<value;
        stream>>agent_size;
        if(agent_size <= 0 || agent_size > 0.5)
        {
            std::cout << "Error! Wrong '"<<CNS_TAG_AGENTSIZE<<"' value found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to '"<<CN_AGENT_SIZE<<"'."<<std::endl;
            agent_size = CN_AGENT_SIZE;
        }
        stream.clear();
        stream.str("");
    }

    element = algorithm->FirstChildElement(CNS_TAG_TIMELIMIT);
    if (!element)
    {
        std::cout << "Error! No '"<<CNS_TAG_TIMELIMIT<<"' element found inside '"<<CNS_TAG_ALGORITHM<<"' section. It's compared to '"<<CN_TIMELIMIT<<"'."<<std::endl;
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
