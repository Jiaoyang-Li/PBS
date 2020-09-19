#ifndef COMPUTEHEURISTIC_H
#define COMPUTEHEURISTIC_H

#include <vector>
#include <utility>
#include <stdlib.h>
#include "egraph_reader.h"

using namespace std;

class ComputeHeuristic {
 public:
  int start_location;
  int goal_location;
  int agent_size;
  const bool* my_map;
  int map_rows;
  int map_cols;
  const int* moves_offset;
  const int* actions_offset;
  const EgraphReader* egr;
  double e_weight;
  
  ComputeHeuristic(int start_location, int goal_location, const bool* my_map, int map_rows, int map_cols, const int* moves_offset,
                   const int* actions_offset, double e_weight, const EgraphReader* egr);
  
  double* getHVals(); 
  double* getEstimatedGVals(); 

  void getHVals(vector<int>& res);

  ~ComputeHeuristic();

  private:
	double* getShortestPathVals(int root);
};

#endif
