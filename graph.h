#ifndef GRAPH_H
#define GRAPH_H
#include <string>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <vector>
#include "tinyxml2.h"
#include "const.h"
#include "structs.h"

class Graph
{
    std::vector<gNode> nodes;
public:
    Graph(){}
    bool get_graph(const char* FileName);
    void check_distances();
    int get_nodes_size() const { return nodes.size(); }
    gNode get_gnode(int id) const
    {
        if(id >= 0 && id < nodes.size())
            return nodes[id];
        else
            return gNode();
    }
    std::vector<Node> get_valid_moves(int id) const
    {
        gNode cur = nodes[id];
        Node node;
        std::vector<Node> neighbors;
        neighbors.clear();
        for(int i = 0; i < cur.neighbors.size(); i++)
        {
            node.i = nodes[cur.neighbors[i]].i;
            node.j = nodes[cur.neighbors[i]].j;
            node.id = cur.neighbors[i];
            neighbors.push_back(node);
        }
        return neighbors;
    }
    void generate_task(int id, int k);
};

#endif // GRAPH_H
