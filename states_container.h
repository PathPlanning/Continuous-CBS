#ifndef multi_index_H
#define multi_index_H
#include "structs.h"
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/composite_key.hpp>
#include <boost/tuple/tuple.hpp>
#include <fstream>
#include <iostream>
#include "lineofsight.h"
#include "map.h"
#include "pheuristic.h"

using namespace boost::multi_index;
struct by_ij;
struct by_fg;
struct by_cons;
typedef multi_index_container<
    oNode,
    indexed_by<
        ordered_unique<
            tag<by_ij>,
            composite_key<
                oNode,
                member<oNode, int, &oNode::i>,
                member<oNode, int, &oNode::j>,
                member<oNode, int, &oNode::interval_id>
            >
        >,
        ordered_non_unique<
            tag<by_fg>,
            composite_key<
                oNode,
                member<oNode, bool,   &oNode::expanded>,
                member<oNode, double, &oNode::F>,
                member<oNode, double, &oNode::h>,
                member<oNode, int, &oNode::id>
            >
        >,
        ordered_non_unique<
            tag<by_cons>,

                member<oNode, int, &oNode::consistent>

        >

    >
> multi_index;

class StatesContainer
{
public:
    const Map* map;
    multi_index states;
    struct updateExpand
    {
        updateExpand(double best_g, bool expanded):best_g(best_g),expanded(expanded){}
        void operator()(oNode& n)
        {
            n.expanded = expanded;
            n.Parent = n.best_Parent;
            n.F = best_g + n.h;
            n.g = best_g;
            n.best_g = best_g;
            n.consistent = 1;
        }

    private:
        bool expanded;
        double best_g;
    };
    struct addParent
    {
        addParent(const oNode* parent, double new_g):parent(parent), new_g(new_g){}
        void operator()(oNode &n)
        {
            n.parents.push_back({parent, new_g});
        }
        private:
            const oNode* parent;
            double new_g;
    };

    struct updateFG
    {
        updateFG(double g, const oNode* parent):g(g), parent(parent){}
        void operator()(oNode& n)
        {
            n.F = g + n.h;
            n.g = g;
            if(n.consistent == 0)
                n.consistent = 2;
            n.Parent = parent;
        }

    private:
        double g;
        const oNode* parent;
    };

    struct updateBest
    {
        updateBest(double g, const oNode* parent):g(g),parent(parent){}
        void operator()(oNode& n)
        {
            n.best_Parent = parent;
            n.best_g = g;
        }
    private:
        double g;
        const oNode* parent;
    };


    struct pop_parent
    {
        pop_parent(const oNode* parent):parent(parent){}
        void operator()(oNode& n)
        {
            for(auto it = n.parents.begin(); it != n.parents.end(); it++)
                if(it->first->id == parent->id && it->first->interval_id == parent->interval_id)
                {
                    n.parents.erase(it);
                    return;
                }
        }
    private:
        const oNode* parent;
    };

    oNode getMin()
    {
        typedef multi_index::index<by_fg>::type fg_index;
        fg_index & fg = states.get<by_fg>();
        return *(fg.begin());
    }

    void expand(const oNode& curNode)
    {
        typedef multi_index::index<by_ij>::type ij_index;
        ij_index & ij = states.get<by_ij>();
        auto it = ij.find(boost::tuple<int, int, int>(curNode.i, curNode.j, curNode.interval_id));
        ij.modify(it, updateExpand(curNode.best_g, true));
    }

    const oNode* getParentPtr()
    {
        typedef multi_index::index<by_fg>::type fg_index;
        fg_index & fg = states.get<by_fg>();
        return &(*(fg.begin()));
    }

    std::vector<const oNode*> getAllParentsPtr()
    {
        typedef multi_index::index<by_fg>::type fg_index;
        fg_index & fg = states.get<by_fg>();
        std::vector<const oNode*> parents;
        for(auto it = fg.begin(); it!= fg.end(); it++)
            parents.push_back(&(*it));
        return parents;
    }

    void insert(const oNode& curNode)
    {
        states.insert(curNode);
    }

    void update(oNode curNode, bool best, PHeuristic* h)
    {
        typedef multi_index::index<by_ij>::type ij_index;
        ij_index & ij = states.get<by_ij>();
        auto it = ij.find(boost::tuple<int, int, int>(curNode.i, curNode.j, curNode.interval_id));
        ij.modify(it, pop_parent(curNode.Parent));
        ij.modify(it, updateFG(curNode.best_g, curNode.best_Parent));
        if(best)
            ij.modify(it, updateBest(curNode.best_g, curNode.best_Parent));
        if(curNode.consistent == 0)
        {
            auto parents = this->findParents(curNode, h);
            for(auto pit = parents.begin(); pit!= parents.end(); pit++)
                ij.modify(it, addParent(pit->first, pit->second));
        }
        if(!it->parents.empty())
        {
            const oNode* new_parent;
            double min_g = CN_INFINITY;
            for(auto pit = it->parents.begin(); pit!= it->parents.end(); pit++)
                if(pit->second < min_g)
                {
                    min_g = pit->second;
                    new_parent = &(*pit->first);
                }
            if(min_g < curNode.best_g)
                ij.modify(it, updateFG(min_g, new_parent));
        }
    }

    std::list<std::pair<const oNode*, double>> findParents(const oNode& curNode, PHeuristic* h)
    {
        std::list<std::pair<const oNode*, double>> parents;
        parents.clear();
        typedef multi_index::index<by_cons>::type cons_index;
        cons_index & cons = states.get<by_cons>();
        auto range = cons.equal_range(1);
        double dist;
        //bool found = false;
        for(auto it = range.first; it != range.second; it++)
        {
            /*if(!found)
                if(it->i == curNode.Parent->i && it->j == curNode.Parent->j && it->interval_id == curNode.Parent->interval_id)
                {
                    //std::cout<<curNode.Parent->i<<" "<<curNode.Parent->j<<" found current parent\n";
                    found = true;
                    it++;
                    if(it == range.second)
                        break;
                }*/
            dist = sqrt(pow(it->i - curNode.i,2) + pow(it->j - curNode.j,2));
            if(it->g + dist < curNode.best_g)
            {
                if(it->g + dist >= curNode.interval.begin)
                {
                    if(it->g + dist <= curNode.interval.end)
                    {
                        if(!h->get_los(curNode.i, curNode.j, it->i, it->j, *map))
                            continue;
                        parents.push_back({&(*it), it->g + dist});
                    }
                }
                else if(it->interval.end + dist >= curNode.interval.begin)
                {
                    if(!h->get_los(curNode.i, curNode.j, it->i, it->j, *map))
                        continue;
                    parents.push_front({&(*it), curNode.interval.begin});
                }
            }
        }
        return parents;
    }

    void updateNonCons(const oNode& curNode, PHeuristic* h)
    {
        typedef multi_index::index<by_ij>::type ij_index;
        ij_index & ij = states.get<by_ij>();
        auto parent = &(*ij.find(boost::tuple<int, int, int>(curNode.i, curNode.j, curNode.interval_id)));
        typedef multi_index::index<by_cons>::type nc_index;
        nc_index & non_cons = states.get<by_cons>();
        auto range = non_cons.equal_range(2);
        double dist, new_g;
        for(auto it = range.first; it != range.second; it++)
        {
            dist = sqrt(pow(it->i - curNode.i,2) + pow(it->j - curNode.j,2));
            new_g = dist + curNode.best_g;
            if(new_g < it->best_g)
            {
                if(new_g >= it->interval.begin)
                {
                    if(new_g <= it->interval.end)
                    {
                        if(!h->get_los(parent->i, parent->j, it->i, it->j, *map))
                            continue;
                        non_cons.modify(it, addParent(parent, new_g));
                        if(it->g - new_g > CN_EPSILON)
                            non_cons.modify(it, updateFG(new_g, parent));
                    }
                }
                else if(curNode.interval.end + dist >= it->interval.begin)
                {
                    if(!h->get_los(parent->i, parent->j, it->i, it->j, *map))
                        continue;
                    non_cons.modify(it, addParent(parent, it->interval.begin));
                    non_cons.modify(it, updateFG(it->interval.begin, parent));
                }
            }
        }
    }

    std::vector<oNode> get_nodes_by_ij(int i, int j)
    {
        std::vector<oNode> nodes;
        typedef multi_index::index<by_ij>::type ij_index;
        ij_index & ij = states.get<by_ij>();
        auto range = ij.equal_range(boost::make_tuple(i,j));
        for(auto it = range.first; it != range.second; it++)
            nodes.push_back(*it);
        return nodes;
    }

    void findAndPrint(int i, int j)
    {
        typedef multi_index::index<by_ij>::type ij_index;
        ij_index & ij = states.get<by_ij>();
        auto range = ij.equal_range(boost::make_tuple(i,j));
        for(auto it = range.first; it != range.second; it++)
        {
            std::cout<<it->i<<" "<<it->j<<" "<<it->g<<" "<<it->F<<" "<<it->best_g<<"\n";
            for(auto p_it = it->parents.begin(); p_it!=it->parents.end(); p_it++)
                std::cout<<p_it->first->i<<" "<<p_it->first->j<<" "<<p_it->first->g<<" "<<p_it->first->F<<" "<<p_it->first->best_g<<" "<<p_it->second<<" parent\n";
        }
    }

    void printByFG()
    {
        typedef multi_index::index<by_fg>::type fg_index;
        fg_index & fg = states.get<by_fg>();
        for(auto it=fg.begin(); it!=fg.end(); it++)
            std::cout<<it->i<<" "<<it->j<<" "<<it->interval.begin<<" "<<it->interval.end<<" "<<it->g<<" "<<it->F<<" "<<it->best_g<<" "<<it->parents.size()<<" "<<it->expanded<<" "<<it->consistent<<"\n";
    }
    void printStats()
    {
        typedef multi_index::index<by_cons>::type cons_index;
        cons_index & cons = states.get<by_cons>();
        std::cout<<cons.count(0)<<" "<<cons.count(1)<<" "<<cons.count(2)<<" STATS\n";
    }

    void clear()
    {
        states.clear();
    }

    int size()
    {
        return states.size();
    }

    int getUncheked()
    {
        typedef multi_index::index<by_cons>::type cons_index;
        cons_index & cons = states.get<by_cons>();
        return cons.count(0);
    }

    int getConsistent()
    {
        typedef multi_index::index<by_cons>::type cons_index;
        cons_index & cons = states.get<by_cons>();
        return cons.count(1);
    }

    int getInconsistent()
    {
        typedef multi_index::index<by_cons>::type cons_index;
        cons_index & cons = states.get<by_cons>();
        return cons.count(2);
    }
};


#endif // multi_index_H
