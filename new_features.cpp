#include "cbs.h"

/*std::pair<std::vector<Move>,std::vector<Move>> CBS::find_similar_actions(Move a, Move b)
{

    std::vector<Move> moves_a = {a};
    std::vector<Move> moves_b = {b};
    std::vector<std::pair<int, int>> deltas = {{1,1}, {1,-1}, {-1,-1}, {-1,1}, {1,0}, {0,1}, {-1,0}, {0,-1}};
    for(auto d:deltas)
    {
        int id2 = map->get_id(map->get_i(a.id2) + d.first, map->get_j(a.id2) + d.second);
        Move new_move = Move(a.t1, a.t1+dist(a.id1, id2), a.id1, id2);
        bool has_col = true;
        Constraint con;
        for(auto c:moves_b)
            if(!check_conflict(new_move, c))
            {
                has_col = false;
                break;
            }
            else
            {
                con = get_constraint(0, new_move, c);
            }
        if(has_col && con.t2 > a.t2)
            moves_a.push_back(new_move);
    }
    for(auto d:deltas)
    {
        int id2 = map->get_id(map->get_i(b.id2) + d.first, map->get_j(b.id2) + d.second);
        Move new_move = Move(b.t1, b.t1+dist(b.id1, id2), b.id1, id2);
        bool has_col = true;
        Constraint con;
        for(auto c:moves_a)
            if(!check_conflict(new_move, c))
            {
                //std::cout<<"no collision "<<moves_a.size()<<" "<<moves_b.size()<<"\n";
                has_col = false;
                break;
            }
            else
            {
                con = get_constraint(0, new_move, c);
            }
        if(has_col && con.t2 > b.t2)
            moves_b.push_back(new_move);
    }
    return {moves_a, moves_b};
}*/

std::pair<std::vector<Move>,std::vector<Move>> CBS::find_similar_actions(Move a, Move b)
{
    std::vector<Move> moves_a = {a};
    std::vector<Move> moves_b = {b};
    LineOfSight los;
    auto cells_a = los.getCellsCrossedByLine(map->get_i(a.id1), map->get_j(a.id1), map->get_i(a.id2), map->get_j(a.id2), *map);
    auto cells_b = los.getCellsCrossedByLine(map->get_i(b.id1), map->get_j(b.id1), map->get_i(b.id2), map->get_j(b.id2), *map);
    int i(1);
    std::ofstream out;
    out.open("log_moves.xml", std::ios::app);
    out<<"\n\n";
    while(i < cells_a.size() or i < cells_b.size())
    {
        if(i < cells_a.size())
        {
            int id_a = map->get_id(cells_a[cells_a.size() - i].first, cells_a[cells_a.size() - i].second);
            Move new_move = Move(a.t1, a.t1+dist(a.id1, id_a), a.id1, id_a);
            bool has_col = true;
            Constraint con;
            for(auto c:moves_b)
                if(!check_conflict(new_move, c))
                {
                    has_col = false;
                    break;
                }
            if(has_col)
            {
                moves_a.push_back(new_move);
                out<<"\t\t\t<constraint i1=\""<<map->get_i(a.id1)<<"\" j1=\""<<map->get_j(a.id1)<<"\" i2=\""<<map->get_i(id_a)<<"\" j2=\""<<map->get_j(id_a)<<"\" t1=\""<<a.t1<<"\" c.t2=\""<<a.t2<<"\" positive=\""<<false<<"\" agent_id=\""<<0<<"\"/>\n";
            }
        }
        if(i < cells_b.size())
        {
            int id_b = map->get_id(cells_b[cells_b.size() - i].first, cells_b[cells_b.size() - i].second);
            Move new_move = Move(b.t1, b.t1+dist(b.id1, id_b), b.id1, id_b);
            bool has_col = true;
            for(auto c:moves_a)
                if(!check_conflict(new_move, c))
                {
                    has_col = false;
                    break;
                }
            if(has_col)
            {
                out<<"\t\t\t<constraint i1=\""<<map->get_i(b.id1)<<"\" j1=\""<<map->get_j(b.id1)<<"\" i2=\""<<map->get_i(id_b)<<"\" j2=\""<<map->get_j(id_b)<<"\" t1=\""<<a.t1<<"\" c.t2=\""<<a.t2<<"\" positive=\""<<false<<"\" agent_id=\""<<0<<"\"/>\n";
                moves_b.push_back(new_move);
            }
        }
        i++;
    }
    out.close();
    //std::cout<<moves_a.size()<<" "<<moves_b.size()<<" moves found\n";
    return {moves_a, moves_b};
}

std::pair<std::vector<Move>,std::vector<Move>> CBS::find_similar_actions(const Task &task, Conflict conflict, std::list<Constraint> consA, std::list<Constraint> consB)
{
    std::vector<Move> moves_a = {conflict.move1};
    std::vector<Move> moves_b = {conflict.move2};
    Constraint constraintA(get_constraint(conflict.agent1, conflict.move1, conflict.move2));
    Constraint constraintB(get_constraint(conflict.agent2, conflict.move2, conflict.move1));
    double durA = constraintA.t2 - constraintA.t1;
    double durB = constraintB.t2 - constraintB.t1;
    while(true)
    {
        consA.push_back(constraintA);
        consB.push_back(constraintB);
        sPath pathA = sPath(aa_planner.find_path(task.get_agent(conflict.agent1), *map, consA, aa_h_values[conflict.agent1]));
        sPath pathB = sPath(aa_planner.find_path(task.get_agent(conflict.agent2), *map, consA, aa_h_values[conflict.agent2]));
        Conflict new_c = check_paths(pathA, pathB);
        //std::cout<<new_c.move1.id1<<" "<<new_c.move1.id2<<" vs "<<new_c.move2.id1<<" "<<new_c.move2.id2<<"\n";
        if(new_c.agent1 >= 0)
        {
            Move new_move = new_c.move1;
            bool has_col = true;
            Constraint con;
            double least_durA(durA), least_durB(durB);
            for(auto c:moves_b)
                if(!check_conflict(new_move, c))
                {
                    has_col = false;
                    break;
                }
                else
                {
                    con = get_constraint(0, new_move, c);
                    least_durA = std::fmin(least_durA, con.t2 - con.t1);
                }
            if(has_col && least_durA > durA/2)
                moves_a.push_back(new_move);
            new_move = new_c.move2;
            has_col = true;
            for(auto c:moves_a)
                if(!check_conflict(new_move, c))
                {
                    has_col = false;
                    break;
                }
                else
                {
                    con = get_constraint(0, new_move, c);
                }
            if(has_col && con.t2 - con.t1 > durB/2)
                moves_b.push_back(new_move);
            constraintA = Constraint(conflict.agent1, 0, CN_INFINITY, new_c.move1.id1, new_c.move1.id2);
            constraintB = Constraint(conflict.agent2, 0, CN_INFINITY, new_c.move2.id1, new_c.move2.id2);
        }
        else
            break;
    }
    return {moves_a, moves_b};
}
