#include <iostream>
#include <fstream>
#include "map.h"
#include "task.h"
#include "cbs.h"
#include "xml_logger.h"

int main(int argc, const char *argv[])
{

    /*std::string task_name = "empty-16-16-random-24.xml";
    Config config;
    Map map = Map(config.agent_size, config.connectdness);
    std::string map_name = "map.xml";
    //std::cout<<map_name<<" map\n";
    map.get_map(map_name.c_str());
    Task task;
    //std::cout<<"task\n";
    task.get_task(task_name.c_str());
    if(map.is_roadmap())
        task.make_ij(map);
    else
        task.make_ids(map.get_width());
    TO_AA_SIPP planner;
    PHeuristic h_values;
    h_values.count(map, task.get_agent(0));
    Multiconstraint m;
    m.agent = 0;
    m.positive = true;
    m.constraints.push_back(Constraint(0, 0, 10, map.get_id(2,2), map.get_id(3,3), true));
    auto path = planner.find_path(task.get_agent(0), map, {m}, h_values);
    std::cout<<path.cost<<" cost\n";
    for(auto n:path.nodes)
        std::cout<<n.i<<" "<<n.j<<" "<<n.g<<" "<<n.interval.first<<" "<<n.interval.second<<" "<<n.parent<<"\n";*/

    std::vector<std::string> collections = {"empty-16-16-random"};//, "room-64-64-8-random",  "den520d-random",  "warehouse-10-20-10-2-2-random"};
    for(auto col:collections)
    for(int k = 12; k <= 25; k++)
        for(int ag=10; ag<100; ag++)
        {
            std::string tname = col + '/' + col + '-' + std::to_string(k) +".xml";
            Config config;
            //std::cout<<"config\n";
            //config.getConfig("config.xml");
            Map map = Map(config.agent_size, config.connectdness);
            std::string map_name = col+"/map.xml";
            //std::cout<<map_name<<" map\n";
            map.get_map(map_name.c_str());
            Task task;
            //std::cout<<"task\n";
            task.get_task(tname.c_str(), ag);
            if(map.is_roadmap())
                task.make_ij(map);
            else
                task.make_ids(map.get_width());
            CBS cbs;
            Solution solution = cbs.find_solution(map, task, config);
            auto found = solution.found?"true":"false";
            std::cout<< ag<<" Soulution found: " << found << "\nRuntime: "<<solution.time.count() << "\nMakespan: " << solution.makespan << "\nFlowtime: " << solution.flowtime<< "\nInitial Cost: "<<solution.init_cost<< "\nCollision Checking Time: " << solution.check_time
                     << "\nHL expanded: " << solution.high_level_expanded << "\nLL searches: " << solution.low_level_expansions << "\nLL expanded(avg): " << solution.low_level_expanded << std::endl;
            std::ofstream out("results_aacbs_naive_empty.txt", std::ios::app);
            out<<k<<" "<<ag<<" "<<solution.found<<" "<<solution.time.count()<<" "<<solution.makespan<<" "<<solution.flowtime<<" "<<solution.init_cost<<" "<<solution.initial_conflicts<<" "<<solution.check_time<<" "<<solution.high_level_expanded<<" "<<solution.low_level_expansions<<" "<<solution.low_level_expanded<<"\n";

            out.close();
            if(!solution.found)
                ag = 100;
        }
    if(argc > 2)
    {
        Config config;
        if(argc > 3)
            config.getConfig(argv[3]);
        Map map = Map(config.agent_size, config.connectdness);
        map.get_map(argv[1]);
        Task task;
        task.get_task(argv[2]);
        if(map.is_roadmap())
            task.make_ij(map);
        else
            task.make_ids(map.get_width());
        CBS cbs;
        Solution solution = cbs.find_solution(map, task, config);
        XML_logger logger;
        auto found = solution.found?"true":"false";
        std::cout<< "Soulution found: " << found << "\nRuntime: "<<solution.time.count() << "\nMakespan: " << solution.makespan << "\nFlowtime: " << solution.flowtime<< "\nInitial Cost: "<<solution.init_cost<< "\nCollision Checking Time: " << solution.check_time
             << "\nHL expanded: " << solution.high_level_expanded << "\nLL searches: " << solution.low_level_expansions << "\nLL expanded(avg): " << solution.low_level_expanded << std::endl;
        std::cout<<argv[2]<<" log\n";
        logger.get_log(argv[2]);
        logger.write_to_log_summary(solution);
        logger.write_to_log_path(solution, map);
        logger.save_log();
    }
    else
    {
        std::cout<<"Error! Not enough input parameters are specified!\n";
    }
    return 0;
}
