// Load's agents' init and goal states.
// First line: number of agents
// Second line and onward, (x_init,y_init),(x_goal,y_goal) of each agent (one per line)

#ifndef AGENTSLOADER_H
#define AGENTSLOADER_H

#include <string>
#include <vector>
#include <utility>
#include "map_loader.h"

using namespace std;

class AgentsLoader {
 public:
  int num_of_agents;
  vector< pair<int, int> > initial_locations;
  vector< pair<int, int> > goal_locations;
  vector<double> max_v;  // entry [i] is the max translational velocity for agent i
  vector<double> max_w;  // entry [i] is the max rotational velocity for agent i
  vector<double> max_a;  // entry [i] is the max accelration for agent i
  AgentsLoader(const std::string fname, const MapLoader &ml, int agentsNum);
  AgentsLoader();
  void addAgent ( int start_row, int start_col, int goal_row, int goal_col );
  void printAgentsInitGoal ();
  void saveToFile(const std::string fname);
  pair<int, int> agentStartOrGoalAt(int row, int col);
  void clearLocationFromAgents(int row, int col);
  ~AgentsLoader();
};

#endif
