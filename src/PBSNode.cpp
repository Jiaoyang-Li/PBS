#include "PBSNode.h"


void PBSNode::clear()
{
	conflicts.clear();
}

void PBSNode::printConstraints(int id) const
{
    auto curr = this;
    while (curr->parent != nullptr)
    {
        cout << curr->constraint.high << ">" << curr->constraint.low << endl;
        curr = curr->parent;
    }
}

std::ostream& operator<<(std::ostream& os, const PBSNode& node)
{
	os << "Node " << node.time_generated << " ( cost = " << node.cost << ", conflicts = " << node.conflicts.size() <<
        " ) with " << node.getNumNewPaths() << " new paths ";
	return os;
}