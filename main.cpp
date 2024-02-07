#include <iostream>
#include "map.h"
#include "task.h"
#include "cbs.h"
#include "xml_logger.h"

int main(int argc, const char *argv[])
{
    Config config;
    std::string map_name("map.xml"), task_name("task.xml"), config_name("config.xml");
    if(argc > 1)
        map_name = argv[1];
    if(argc > 2)
        task_name = argv[2];
    if(argc > 3)
        config_name = argv[3];
    config.getConfig(config_name.c_str());
    Task task;
    bool got_task = task.get_task(task_name.c_str());
    Map map = Map(config.agent_size);
    bool got_map = map.get_map(map_name.c_str());
    if(!got_task || !got_map)
    {
        std::cout<<"Not enough input data!\n";
        return 0;
    }

    task.make_ids(map.get_width());
    PHeuristic h_values;
    h_values.init(map.get_width(), map.get_height());
    h_values.count(map);

    CBS cbs;
    Solution solution = cbs.find_solution(map, task, config, h_values);
    auto found = solution.found?"true":"false";
    std::cout<<"Soulution found: " << found << "\nRuntime: "<<solution.time.count() << "\nMakespan: " << solution.makespan << "\nFlowtime: " << solution.flowtime<< "\nInitial Cost: "<<solution.init_cost<< "\nCollision Checking Time: " << solution.check_time
              << "\nHL expanded: " << solution.high_level_expanded << "\nLL searches: " << solution.low_level_expansions << "\nLL expanded(avg): " << solution.low_level_expanded << std::endl;

    XML_logger logger;
    logger.get_log(map_name.c_str());
    logger.write_to_log_summary(solution);
    logger.write_to_log_path(solution, map);
    logger.save_log();

    return 0;
}