#include <iostream>
#include <fstream>
#include "map.h"
#include "task.h"
#include "cbs.h"
#include "xml_logger.h"

int main(int argc, const char *argv[])
{
    Config config;
    if(argc > 1)
        config.getConfig(argv[1]);
    else
        config.getConfig("config.xml");
    Map map = Map(config.agent_size, config.connectdness);
    std::string map_name = "map.xml";
    map.get_map(map_name.c_str());
    PHeuristic h_values;
    h_values.init(map.get_width(), map.get_height());
    h_values.count(map);

    Task task;
    task.get_task("task.xml");
    task.make_ids(map.get_width());
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

    std::vector<std::string> collections = {"warehouse-10-20-10-2-2", "random-32-32-20", "den312d", "maze-32-32-4", "empty-16-16"};
    std::vector<std::string> configs = {"config_basic_1.0.xml","config_ds_1.0.xml", "config_dump.xml", "config_mc_half.xml", "config_ds_mc_1.0.xml", "config_mc_1.0.xml",
                                         "config_ds_1.01.xml", "config_ds_mc_1.01.xml", "config_mc_1.01.xml", "config_basic_1.01.xml",
                                         "config_ds_mc_1.1.xml", "config_mc_1.1.xml", "config_ds_1.1.xml", "config_basic_1.1.xml",
                                        "config_ds_1.25.xml", "config_basic_1.25.xml", "config_ds_mc_1.25.xml", "config_mc_1.25.xml"};
    for(auto col:collections)
    {
        Config config;
        if(argc > 1)
            config.getConfig(argv[1]);
        else
            config.getConfig("config.xml");
        Map map = Map(config.agent_size, config.connectdness);
        std::string map_name = col+"/map.xml";
        map.get_map(map_name.c_str());
        PHeuristic h_values;
        h_values.init(map.get_width(), map.get_height());
        h_values.load(col + '/' +col, map);
        for(int k = 1; k <= 25; k++)
            for(int ag = 2; ag <= 100; ag++)
            {
                std::string tname = col + '/' + col + "-random-" + std::to_string(k) +".xml";
                Task task;
                task.get_task(tname.c_str(), ag);
                if(map.is_roadmap())
                    task.make_ij(map);
                else
                    task.make_ids(map.get_width());
                CBS cbs;
                Solution solution = cbs.find_solution(map, task, config, h_values);
                auto found = solution.found?"true":"false";
                std::cout<< ag<<" Soulution found: " << found << "\nRuntime: "<<solution.time.count() << "\nMakespan: " << solution.makespan << "\nFlowtime: " << solution.flowtime<< "\nInitial Cost: "<<solution.init_cost<< "\nCollision Checking Time: " << solution.check_time
                         << "\nHL expanded: " << solution.high_level_expanded << "\nLL searches: " << solution.low_level_expansions << "\nLL expanded(avg): " << solution.low_level_expanded << std::endl;
                std::string out_name = col + "-results-" + std::to_string(config.mc_type)+"-"+ std::to_string(config.use_disjoint_splitting)+"-"+ std::to_string(config.focal_weight)+".log";
                std::ofstream out(out_name, std::ios::app);
                out<<k<<" "<<ag<<" "<<solution.found<<" "<<solution.time.count()<<" "<<solution.makespan<<" "<<solution.flowtime<<" "<<solution.init_cost<<" "<<solution.initial_conflicts<<" "<<solution.check_time<<" "<<solution.high_level_expanded<<" "<<solution.low_level_expansions<<" "<<solution.low_level_expanded<<"\n";
                out.close();
                /*XML_logger logger;
                logger.get_log(map_name.c_str());
                logger.write_to_log_summary(solution);
                logger.write_to_log_path(solution, map);
                logger.save_log();*/
                if(!solution.found)
                    break;
            }
    }
    return 0;
}
