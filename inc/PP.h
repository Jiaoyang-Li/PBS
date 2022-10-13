#pragma once
#include "SingleAgentSolver.h"

class PP
{
public:
	/////////////////////////////////////////////////////////////////////////////////////
	// stats
	clock_t runtime = 0;
	clock_t runtime_generate_child = 0; // runtimr of generating child nodes
	clock_t runtime_build_CT = 0; // runtimr of building constraint table
	clock_t runtime_build_CAT = 0; // runtime of building conflict avoidance table
	clock_t runtime_path_finding = 0; // runtime of finding paths for single agents
	clock_t runtime_detect_conflicts = 0;
	clock_t runtime_preprocessing = 0; // runtime of building heuristic table for the low level

	uint64_t num_HL_expanded = 0;
	uint64_t num_HL_generated = 0;
	uint64_t num_LL_expanded = 0;
	uint64_t num_LL_generated = 0;
	uint64_t num_restart = 0;
	uint64_t num_LL_search_calls = 0;

	bool solution_found = false;
	int solution_cost = -2;

	PP(const Instance& instance, bool sipp, int screen, bool use_LH=false);
	~PP(){}

	// Runs the algorithm until the problem is solved or time is exhausted 
	bool solve(double time_limit);
	void clearSearchEngines();

	// Save results
	void saveResults(const string &fileName, const string &instanceName) const;
    void savePaths(const string &fileName) const; // write the paths to a file
	void clear(); // used for rapid random  restart

private:
	conflict_selection conflict_seletion_rule;
    string getSolverName() const;

	int screen;
	clock_t time_limit;
	int node_limit = MAX_NODES;
	steady_clock::time_point start;
	int num_of_agents;
	int num_of_cols;
	int map_size;
	bool use_LH;

    vector<int> ordered_agents;
	vector<Path*> paths;
	vector <SingleAgentSolver*> search_engines;  // used to find (single) agents' paths and mdd

    int getSumOfCosts() const;

	// print and save
	void printResults() const;

	bool validateSolution() const;
	inline int getAgentLocation(int agent_id, size_t timestep) const;

	// high level search
	void printPaths() const;
	void printAgentPath(int i) const;
};
