#include <iostream>
#include <fstream>
#include "map.h"
#include "task.h"
#include "cbs.h"
#include "xml_logger.h"
int main(int argc, const char *argv[])
{
    if(argc > 2)
    {
        Map map = Map();
        map.get_map(argv[1]);
        map.generate_moves();
        Task task;
        task.get_task(argv[2]);
        CBS cbs;
        Solution solution = cbs.find_solution(map, task);
        XML_logger logger;
        std::cout<< "Runtime: "<<solution.time.count() << "\nMakespan: " << solution.makespan << "\nFlowtime:" << solution.flowtime<< "\nInitial Cost: "<<solution.init_cost<< "\nCollision Checking Time: " << solution.check_time
             << "\nHL expanded: " << solution.high_level_expanded << "\nLL searches: " << solution.low_level_expansions << "\nLL expanded(avg): " << solution.low_level_expanded << std::endl;

        logger.get_log(argv[2]);
        logger.write_to_log_summary(solution);
        logger.write_to_log_path(solution);
        logger.save_log();
    }
    else
    {
        std::cout<<"Error! Not enough input parameters are specified!\n";
    }
    return 0;

    /*std::ofstream out;
    out.open("log_den_k3_focal.txt");
    auto agents = {10,15,20,25,30};
    for(auto a: agents)
        for(int i = 0; i < 250; i++)
    {
        auto map_path = "D:/Users/andreychuk/Documents/GitHub/CBS/build-CBS-SIPP-Desktop_Qt_5_10_1_MinGW_32bit-Release/release/instances/den520d/map.xml";
        auto task_path = "D:/Users/andreychuk/Documents/GitHub/CBS/build-CBS-SIPP-Desktop_Qt_5_10_1_MinGW_32bit-Release/release/instances/den520d/"+std::to_string(a)+"/"+std::to_string(i)+"_task.xml";
        //std::cout<<map_path<<"\n"<<task_path<<"\n";
        Map map = Map();
        map.get_map(map_path);
        map.generate_moves();
        Task task;
        task.get_task(task_path.c_str());
        CBS cbs;
        Solution solution = cbs.find_solution(map, task);

        out<<a<<" "<<i<<" "<<solution.time.count() << " " << solution.makespan << " " << solution.flowtime<< " "<<solution.init_cost<< " " << solution.init_time.count() << " " <<
                           solution.check_time << " " << solution.high_level_expanded << " " << solution.low_level_expansions << " " << solution.low_level_expanded << " " << solution.cardinal_solved << " "
                     << solution.semicardinal_solved << std::endl;
        std::cout<<a<<" "<<i<<" "<<solution.time.count() << " " << solution.makespan << " " << solution.flowtime<< " "<<solution.init_cost<< " " << solution.init_time.count() << " " <<
                   solution.check_time << " " << solution.high_level_expanded << " " << solution.low_level_expansions << " " << solution.low_level_expanded << " " << solution.cardinal_solved << " "
             << solution.semicardinal_solved << std::endl;
    }
    out.close();

    return 0;*/
}
