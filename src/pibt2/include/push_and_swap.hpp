/*
 * Implementation of Push & Swap
 *
 * - ref
 * Luna, R., & Bekris, K. E. (2011, July).
 * Push and swap: Fast cooperative path-finding with completeness guarantees.
 * In IJCAI (pp. 294-300).
 */

#pragma once
#include "solver.hpp"

class PushAndSwap : public MAPF_Solver
{
public:
  static const std::string SOLVER_NAME;

private:
  bool flg_compress;                // whether to compress solution
  bool disable_dist_init;           // prioritization depending on distance
  grid_pathfinding::Nodes nodes_with_many_neighbors;  // nodes of degree >= 3
  bool emergency_stop;

  // used in occupancy
  static constexpr int NIL = -1;

  // main
  void run();

  // push operation
  bool push(Plan& plan, const int i, grid_pathfinding::Nodes& U, std::vector<int>& occupied_now);

  // swap operation
  bool swap(Plan& plan, const int i, grid_pathfinding::Nodes& U, std::vector<int>& occupied_now);
  bool swap(Plan& plan, const int i, grid_pathfinding::Nodes& U, std::vector<int>& occupied_now,
            std::vector<int>& recursive_list);

  // improve solution quality, see
  Plan compress(const Plan& plan);

  // ---------------------------------------
  // sub procedures

  // push several agents simultaneously
  bool multiPush(Plan& plan, const int r, const int s, const grid_pathfinding::Path& p,
                 std::vector<int>& occupied_now);

  // clear operation
  bool clear(Plan& plan, grid_pathfinding::Node* v, const int r, const int s,
             std::vector<int>& occupied_now);

  // execute swap operation
  void executeSwap(Plan& plan, const int r, const int s,
                   std::vector<int>& occupied_now);

  // resolve operation
  bool resolve(Plan& plan, const int r, const int s, grid_pathfinding::Nodes& U,
               std::vector<int>& occupied_now);

  // ---------------------------------------
  // utilities

  // get nearest empty location
  grid_pathfinding::Node* getNearestEmptyNode(grid_pathfinding::Node* v, std::vector<int>& occupied_now,
                            const grid_pathfinding::Nodes& obs);

  // get the shortest path towards goals
  grid_pathfinding::Path getShortestPath(const int id, grid_pathfinding::Node* s, std::vector<int>& occupied_now);

  // push toward empty node
  bool pushTowardEmptyNode(grid_pathfinding::Node* v, Plan& plan, std::vector<int>& occupied_now,
                           const grid_pathfinding::Nodes& obs);

  // update plan
  void updatePlan(const int id, grid_pathfinding::Node* next_node, Plan& plan,
                  std::vector<int>& occupied_now);

  // get all vertices of degree >= 3 on G
  void findNodesWithManyNeighbors();

  // error check
  void checkConsistency(Plan& plan, std::vector<int>& occupied_now);

public:
  PushAndSwap(MAPF_Instance* _P);
  ~PushAndSwap() {}

  void setParams(int argc, char* argv[]);
  static void printHelp();
};
