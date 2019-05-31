#include "graph.h"

bool Graph::get_graph(const char *FileName)
{
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(FileName) != tinyxml2::XMLError::XML_SUCCESS)
    {
        std::cout << "Error opening XML file!" << std::endl;
        return false;
    }
    tinyxml2::XMLElement *root = 0, *element = 0, *data;
    std::string value;
    std::stringstream stream;
    root = doc.FirstChildElement("graphml")->FirstChildElement("graph");
    for(element = root->FirstChildElement("node"); element; element = element->NextSiblingElement("node"))
    {
        data = element->FirstChildElement();

        stream.str("");
        stream.clear();
        stream << data->GetText();
        stream >> value;
        auto it = value.find_first_of(",");
        stream.str("");
        stream.clear();
        stream << value.substr(0, it);
        double i;
        stream >> i;
        stream.str("");
        stream.clear();
        value.erase(0, ++it);
        stream << value;
        double j;
        stream >> j;
        gNode node;
        node.i = i;
        node.j = j;
        nodes.push_back(node);
    }
    for(element = root->FirstChildElement("edge"); element; element = element->NextSiblingElement("edge"))
    {
        std::string source = std::string(element->Attribute("source"));
        std::string target = std::string(element->Attribute("target"));
        source.erase(source.begin(),++source.begin());
        target.erase(target.begin(),++target.begin());
        int id1, id2;
        stream.str("");
        stream.clear();
        stream << source;
        stream >> id1;
        stream.str("");
        stream.clear();
        stream << target;
        stream >> id2;
        nodes[id1].neighbors.push_back(id2);
    }

    return true;
}

double dist(const gNode &a, const gNode &b)
{
    return sqrt(pow(a.i - b.i, 2) + pow(a.j - b.j, 2));
}

void Graph::generate_task(int id, int k)
{
    int start_id, goal_id;
    std::vector<int> busy_starts(nodes.size(), 0), busy_goals(nodes.size(), 0);
    std::vector<int> start_ids, goal_ids;
    std::ofstream gggg;
    gggg.open(std::to_string(id)+"_task.xml");
    std::cout<<"<?xml version=\"1.0\" ?>\n<root>\n";
    for(int i = 0; i < k; i++)
    {
        srand(std::chrono::high_resolution_clock::now().time_since_epoch().count());
        start_id = rand()%nodes.size();
        bool valid(false);
        while(!valid)
        {
            while(busy_starts[start_id])
                start_id = rand()%nodes.size();
            valid = true;
            for(int id: start_ids)
                if(dist(nodes[start_id], nodes[id]) < CN_AGENT_SIZE*2)
                {
                    valid = false;
                    busy_starts[start_id] = 1;
                    break;
                }
        }
        busy_starts[start_id] = 1;
        srand(std::chrono::high_resolution_clock::now().time_since_epoch().count()+1000);
        goal_id = rand()%nodes.size();
        valid = false;
        while(!valid)
        {
            while(busy_goals[goal_id])
                goal_id = rand()%nodes.size();
            valid = true;
            for(int id: goal_ids)
                if(dist(nodes[goal_id], nodes[id]) < CN_AGENT_SIZE*2)
                {
                    valid = false;
                    busy_goals[goal_id] = 1;
                    break;
                }
        }
        busy_goals[goal_id] = 1;
        std::cout<<"   <agent start_id=\""<<start_id<<"\" goal_id=\""<<goal_id<<"\"/>\n";
    }
    std::cout<<"</root>";
    gggg.close();
}
