#pragma once

#include "MAPFSolver.h"

// PIBT2 dependencies
#include <graph.hpp>
#include <node.hpp>
#include <paths.hpp>
#include <plan.hpp>
#include <problem.hpp>
#include <util.hpp>
// include grid
// include node
// include pibt2 
#include <pibt_plus.hpp>

class PIBT2Node: public MAPFSolver
{
public:
    PIBT2Node() = default;
    ~PIBT2Node() = default;

    PIBT2Node(const BasicGraph& bG, SingleAgentSolver& path_planner);

    grid_pathfinding::Grid* convertGraphRepresentation(const BasicGraph& bG);
    
    bool run(const std::vector<State>& starts,
    const std::vector<std::vector<std::pair<int, int>>>& goal_locations,
    int time_limit);


    void save_results(const std::string &fileName, const std::string &instanceName) const {};
	void save_search_tree(const std::string &fileName) const {}
	void save_constraints_in_goal_node(const std::string &fileName) const {}
    string get_name() const {return "PIBT2"; }
	void clear() {}

private:
    MAPF_Instance* P_;
    std::unique_ptr<PIBT_PLUS> pibt2_solver;
};

