#ifndef XML_LOGGER_H
#define XML_LOGGER_H
#include "const.h"
#include "tinyxml2.h"
#include "structs.h"
#include "graph.h"
#include <iostream>
#include <sstream>
#include <string>

class XML_logger
{
private:
    std::string LogFileName;
    tinyxml2::XMLDocument *doc;

public:

    XML_logger();
    ~XML_logger() { if(doc) delete doc; }
    bool get_log(const char* FileName);
    void save_log();
    void write_to_log_summary(const Solution &solution);
    void write_to_log_path(const Solution &solution);
    void write_to_log_edges(const Graph &graph);

};

#endif // XML_LOGGER_H
