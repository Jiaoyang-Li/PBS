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
};