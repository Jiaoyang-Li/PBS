#pragma once
#include "common.h"
#include "Conflict.h"

enum node_selection { NODE_RANDOM, NODE_H, NODE_DEPTH, NODE_CONFLICTS, NODE_CONFLICTPAIRS, NODE_MVC };


class PBSNode
{
public:
	Constraint constraint; // new constraint
    list< pair< int, Path> > paths; // new paths
    int cost = 0; // sum of costs

	size_t depth = 0; // depath of this CT node
	size_t makespan = 0; // makespan over all paths

	uint64_t time_expanded = 0;
	uint64_t time_generated = 0;

	// conflicts in the current paths
	list<shared_ptr<Conflict> > conflicts;
	// The chosen conflict
	shared_ptr<Conflict> conflict;

    PBSNode* parent = nullptr;
	PBSNode* children[2] = {nullptr, nullptr};

    PBSNode() = default;
    PBSNode(PBSNode& parent) : cost(parent.cost), depth(parent.depth+1),
                               makespan(parent.makespan), conflicts(parent.conflicts), parent(&parent){ }
	void clear();
	void printConstraints(int id) const;
    inline int getNumNewPaths() const { return (int) paths.size(); }
    inline string getName() const { return "PBS Node"; }
    list<int> getReplannedAgents() const
    {
        list<int> rst;
        for (const auto& path : paths)
            rst.push_back(path.first);
        return rst;
    }
};

std::ostream& operator<<(std::ostream& os, const PBSNode& node);
