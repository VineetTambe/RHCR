#pragma once
#include <graph.hpp>

static int TASK_ID_CNT = 0;

struct Task {
  int id;
  grid_pathfinding::Node* loc_pickup;
  grid_pathfinding::Node* loc_delivery;
  grid_pathfinding::Node* loc_current;
  int timestep_appear;
  int timestep_finished;
  bool assigned;

  static constexpr int NIL = -1;

  Task(grid_pathfinding::Node* loc_p, grid_pathfinding::Node* loc_d, int t)
      : id(TASK_ID_CNT++),
        loc_pickup(loc_p),
        loc_delivery(loc_d),
        loc_current(loc_p),
        timestep_appear(t),
        timestep_finished(NIL),
        assigned(false){};
};

using Tasks = std::vector<Task*>;
