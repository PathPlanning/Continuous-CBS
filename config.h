#ifndef CONFIG_H
#define CONFIG_H
#include "tinyxml2.h"
#include "const.h"
#include <string>
#include <iostream>

class Config
{
public:
    Config();
    bool getConfig(const char* fileName);
    double delta;
    bool cardinal;
    bool history;
    bool stop_cardinal;
    int connectdness;
    double agent_size;
    double timelimit;
};

#endif // CONFIG_H
