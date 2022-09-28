#pragma once
#include "PBS.h"

class PBS2 : public PBS
{
public:
    PBS2(const Instance& instance, bool sipp, int scrren);
    bool solve(double time_limit);

protected:
    string getSolverName() const;
    // shared_ptr<Conflict> chooseConflict(const PBSNode &node) const;

	PBSNode* selectNode();
    // shared_ptr<Conflict> chooseConflict(const PBSNode &node) const;
    // void printConflicts(const PBSNode& node)const;
    // bool terminate(PBSNode* curr);

    // high-level search
    bool generateRoot(void);
    bool generateChild(int child_id, PBSNode* parent, int low, int high);
    int hasConflicts(int a1, int a2) const;
	shared_ptr<Conflict> chooseConflict(const PBSNode &node) const;
};