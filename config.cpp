#include "config.h"
using namespace  tinyxml2;
Config::Config()
{

}


bool Config::getConfig(const char *fileName)
{
    XMLDocument doc;
    if (doc.LoadFile(fileName) != tinyxml2::XMLError::XML_SUCCESS)
    {
        std::cout << "Error opening Config XML file!" << std::endl;
        return false;
    }
    XMLElement *root = doc.FirstChildElement(CNS_TAG_ROOT);
    if (!root)
    {
        std::cout << "No 'root' element found in XML file."<<std::endl;
        return false;
    }


}
