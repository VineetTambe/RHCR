#include "PIBT2Node.hpp"

PIBT2Node::PIBT2Node(const BasicGraph& bG, SingleAgentSolver& path_planner) : MAPFSolver(G, path_planner){
  P_ = new MAPF_Instance(convertGraphRepresentation(bG));
}

grid_pathfinding::Grid* PIBT2Node::convertGraphRepresentation(const BasicGraph& bG) {
  grid_pathfinding::Grid* grid = new grid_pathfinding::Grid();

  grid->setHeight(bG.get_rows());
  grid->setWidth(bG.get_cols());

  grid_pathfinding::Nodes V;
  V = grid_pathfinding::Nodes(bG.size(), nullptr);

  for (int i = 0; i < bG.get_rows(); ++i) {
    for (int j = 0; j < bG.get_cols(); ++j) {
      int id = bG.get_cols() * i + j;
      if (bG.types[id] == "Obstacle") {
        continue;
      }
      grid_pathfinding::Node* v = new grid_pathfinding::Node(id, j, i);
      V[id] = v;

      // std::cout << "Node id: " << id << " x: " << j << " y: " << i << std::endl;
    }
  }

  for (int i = 0; i < bG.get_rows(); ++i) {
    for (int j = 0; j < bG.get_cols(); ++j) {
      int id = bG.get_cols() * i + j;
      if (bG.types[id] == "Obstacle") {
        // std::cout << "Obstacle at id: " << id << std::endl;
        continue;
      }
      for (int dir = 0; dir < 4; dir++) {
        if (0 <= id + bG.move[dir] &&
            id + bG.move[dir] < bG.size() &&
            bG.get_Manhattan_distance(id, id + bG.move[dir]) <= 1 &&
            bG.types[id + bG.move[dir]] != "Obstacle")
          V[id]->neighbor.push_back(V[id + bG.move[dir]]);
      }
    }
  }

  grid->setVertices(V);

  return grid;
}

bool PIBT2Node::run(
    const std::vector<State>& starts,
    const std::vector<std::vector<std::pair<int, int>>>& goal_locations,
    int time_limit) {
    // cout << "Inside PIBT run solve!\n";
    num_of_agents = starts.size();

    P_->setMaxCompTime(time_limit);

    std::vector<int> s;
    std::vector<int> g;

    solution.clear();
	  solution.resize(num_of_agents);

    cout << "Starting solve!\n";

    for(int i = 0 ; i < goal_locations.size(); i++) {
        s.clear();
        g.clear();

        if(i == 0) { 

            for(int i = 0 ; i < starts.size(); i++) {
                s.push_back(starts[i].location);
            }
        } else {
            for(int j = 0 ; j < goal_locations[i-1].size(); j++) {
                s.push_back(goal_locations[i-1][j].first);
            }
        }
        
        // cout << "Setting new start and goal locations!\n";

        for(int j = 0 ; j < goal_locations[i].size(); j++) {
            g.push_back(goal_locations[i][j].first);
        }
        
        P_->setStartsGoals(s, g);

        pibt2_solver = std::make_unique<PIBT_PLUS>(P_);

        // run PIBT2
        pibt2_solver->solve();

        // if unable to get solution return false
        if (pibt2_solver->succeed() && !pibt2_solver->getSolution().validate(P_)) {
          cout << "[PIBT2] Solution not found!\n";
          return false;
        }
        // get solution and append it to the "paths" for agents
        Paths paths = pibt2_solver->planToPaths(pibt2_solver->getSolution());
        for (int agent = 0; agent < num_of_agents; agent++) {
          // Path rhcr_path;
          // cout << "Converting solution to required format!\n";
          for (int t = 0; t < paths.get(agent).size(); t++) {
            // convert pibt2 represnetation to rhcr representation 
            // rhcr_path.push_back(State(paths.get(agent,t)->id, t));
            solution[agent].emplace_back(paths.get(agent,t)->id, t, 0);
          }
          // solution[agent].push_back(rhcr_path);
        }
    }

    return true;
}
