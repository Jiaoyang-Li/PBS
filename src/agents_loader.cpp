//=======================================================================

#include "agents_loader.h"
#include <string>
#include <cstring>
#include <iostream>
#include <cassert>
#include <fstream>
#include<boost/tokenizer.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <utility>
#include <algorithm>  // for remove_if
#include <ctime>
using namespace boost;
using namespace std;

int RANDOM_WALK_STEPS = 100000;

AgentsLoader::AgentsLoader(string fname, const MapLoader &ml, int agentsNum = 0){

  string line;

  ifstream myfile (fname.c_str());

  if (myfile.is_open()) {
    getline (myfile,line);
    if (line[0] == 'v') // Nathan's benchmark
    {
      num_of_agents = agentsNum;
      if (num_of_agents == 0)
      {
          cerr << "The number of agents should be larger than 0" << endl;
          exit(-1);
      }
      initial_locations.resize(num_of_agents);
      goal_locations.resize(num_of_agents);
      char_separator<char> sep("\t");
      for (int i = 0; i < num_of_agents; i++)
      {
          getline(myfile, line);
          tokenizer< char_separator<char> > tok(line, sep);
          tokenizer< char_separator<char> >::iterator beg = tok.begin();
          beg++; // skip the first number
          beg++; // skip the map name
          beg++; // skip the columns
          beg++; // skip the rows
          // read start [row,col] for agent i
          int col = atoi((*beg).c_str());
          beg++;
          int row = atoi((*beg).c_str());
          initial_locations[i] = make_pair(row, col);
          // read goal [row,col] for agent i
          beg++;
          col = atoi((*beg).c_str());
          beg++;
          row = atoi((*beg).c_str());
          goal_locations[i] = make_pair(row, col);
      }
    }
    else // My benchmark
    {
      char_separator<char> sep(",");
      tokenizer< char_separator<char> > tok(line, sep);
      tokenizer< char_separator<char> >::iterator beg = tok.begin();
      num_of_agents = atoi((*beg).c_str());
      initial_locations.resize(num_of_agents);
      goal_locations.resize(num_of_agents);
      for (int i = 0; i<num_of_agents; i++)
      {
          getline(myfile, line);
          tokenizer< char_separator<char> > col_tok(line, sep);
          tokenizer< char_separator<char> >::iterator c_beg = col_tok.begin();
          pair<int, int> curr_pair;
          // read start [row,col] for agent i
          int row = atoi((*c_beg).c_str());
          c_beg++;
          int col = atoi((*c_beg).c_str());
          initial_locations[i] = make_pair(row, col);
          // read goal [row,col] for agent i
          c_beg++;
          row = atoi((*c_beg).c_str());
          c_beg++;
          col = atoi((*c_beg).c_str());
          goal_locations[i] = make_pair(row, col);
      }
    }
    myfile.close();
  } 
  else if(agentsNum > 0)//Generate agents randomly
  {
	  this->num_of_agents = agentsNum;
	  bool* starts = new bool[ml.rows * ml.cols];
	  for (int i = 0; i<ml.rows * ml.cols; i++)
		  starts[i] = false;
	  bool* goals = new bool[ml.rows * ml.cols];
	  for (int i = 0; i<ml.rows * ml.cols; i++)
		  goals[i] = false;
	  // Choose random start locations
	  for (int k = 0; k < agentsNum; k++)
	  {
		  int x = rand() % ml.rows, y = rand() % ml.cols;
		  int start = x * ml.cols +y;
		  if (!ml.my_map[start] && !starts[start])
		  {
				// update start
				this->initial_locations.push_back(make_pair(x,y));
				starts[start] = true;

				// random walk
				int loc = start;
				bool* temp_map = new bool[ml.rows * ml.cols];
				for (int walk = 0; walk < RANDOM_WALK_STEPS; walk++)
				{
					int directions[] = {0, 1, 2, 3, 4};
					random_shuffle(directions, directions + 5);
					int i = 0;
					for(; i< 5; i++)
					{
						int next_loc = loc + ml.moves_offset[directions[i]];
						if (0 <= next_loc && next_loc < ml.rows * ml.cols &&! ml.my_map[next_loc])
						{
							loc = next_loc;
							break;
						}
					}
					if (i == 5)
					{
						cout << "--------------ERROR!-----------------" << endl;
					}
				}
				// find goal
				bool flag = false;
				int goal = loc;
				while (!flag)
				{
					int directions[] = { 0, 1, 2, 3, 4 };
					random_shuffle(directions, directions + 5);
					int i = 0;
					for (; i< 5; i++)
					{
						int next_loc = goal + ml.moves_offset[directions[i]];
						if (0 <= next_loc && next_loc < ml.rows * ml.cols && !ml.my_map[next_loc])
						{
							goal = next_loc;
							break;
						}
					}
					if (i == 5)
					{
						cout << "--------------ERROR!-----------------" << endl;
					}
					flag = true;
					if (goals[goal])
						flag = false;
				}
				//update goal
				this->goal_locations.push_back(make_pair(goal / ml.cols, goal % ml.cols));
				goals[goal] = true;
				// update others
				/*this->max_v.push_back(1);
				this->max_w.push_back(1);
				this->max_a.push_back(1);*/
		  }
		  else
		  {
			  k--;
		  }
	  }
	  saveToFile(fname);
  }
}

void AgentsLoader::printAgentsInitGoal () {
  cout << "AGENTS:" << endl;;
  for (int i=0; i<num_of_agents; i++) {
    cout << "Agent" << i << " : I=(" << initial_locations[i].first << "," << initial_locations[i].second << ") ; G=(" <<
      goal_locations[i].first << "," << goal_locations[i].second << ")" << endl;
  }
  cout << endl;
}

AgentsLoader::~AgentsLoader() {
  // vectors are on stack, so they are freed automatically
}

// create an empty object
AgentsLoader::AgentsLoader() {
  num_of_agents = 0;
}

// returns the agents' ids if they occupy [row,col] (first for start, second for goal)
pair<int, int> AgentsLoader::agentStartOrGoalAt(int row, int col) {
  int f = -1;
  int s = -1;
  for (vector< pair<int, int> >::iterator it = initial_locations.begin(); it != initial_locations.end(); ++it)
    if ( it->first == row && it->second == col )
      f = std::distance(initial_locations.begin(), it);
  for (vector< pair<int, int> >::iterator it = goal_locations.begin(); it != goal_locations.end(); ++it)
    if ( it->first == row && it->second == col )
      s = std::distance(goal_locations.begin(), it);
  return make_pair(f, s);
}


void AgentsLoader::clearLocationFromAgents(int row, int col) {
  pair<int, int> idxs = agentStartOrGoalAt(row, col);
  if ( idxs.first != -1 ) {  // remove the agent who's start is at [row,col]
    initial_locations.erase( initial_locations.begin() + idxs.first );
    goal_locations.erase ( goal_locations.begin() + idxs.first );
    num_of_agents--;
  }
  idxs = agentStartOrGoalAt(row, col);
  if ( idxs.second != -1 ) {  // remove the agent who's goal is at [row,col]
    initial_locations.erase( initial_locations.begin() + idxs.second );
    goal_locations.erase( goal_locations.begin() + idxs.second );
    num_of_agents--;
  }
}


// add an agent
void AgentsLoader::addAgent(int start_row, int start_col, int goal_row, int goal_col) {
  this->initial_locations.push_back(make_pair(start_row, start_col));
  this->goal_locations.push_back(make_pair(goal_row, goal_col));
  num_of_agents++;
}

void AgentsLoader::saveToFile(std::string fname) {
  ofstream myfile;
  myfile.open(fname);
  myfile << num_of_agents << endl;
  for (int i = 0; i < num_of_agents; i++)
    myfile << initial_locations[i].first << "," << initial_locations[i].second << ","
           << goal_locations[i].first << "," << goal_locations[i].second << ","
		   /*<< max_v[i] << "," << max_a[i] << "," << max_w[i] << ","*/  << endl;
  myfile.close();
}
