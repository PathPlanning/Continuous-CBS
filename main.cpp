#include <iostream>
#include "task.h"
#include "cbs.h"
#include "xml_logger.h"
#include <fstream>

int main(int argc, const char *argv[])
{

    if(argc > 2)
    {
        Graph map = Graph();
        map.get_graph(argv[1]);
        Task task;
        task.get_task(argv[2]);
        CBS cbs;
        Solution solution = cbs.find_solution(map, task);
        XML_logger logger;
        std::cout<< "Runtime: "<<solution.time.count() << "\nMakespan: " << solution.makespan << "\nFlowtime:" << solution.flowtime<< "\nInitial Cost: "<<solution.init_cost<< "\nCollision Checking Time: " << solution.check_time
             << "\nHL expanded: " << solution.high_level_expanded << "\nLL searches: " << solution.low_level_expanded << "\nLL expanded(avg): " << solution.low_level_expansions << std::endl;

        logger.get_log(argv[2]);
        logger.write_to_log_summary(solution);
        logger.write_to_log_path(solution);
        logger.write_to_log_edges(map);
        logger.save_log();
    }
    return 0;
}
