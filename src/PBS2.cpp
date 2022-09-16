#include <algorithm>
#include <random>
#include <chrono>
#include "PBS2.h"
#include "SIPP.h"
#include "SpaceTimeAStar.h"

PBS2::PBS2(const Instance& instance, bool sipp, int screen) :
    PBS(instance, sipp, screen) {}

bool PBS2::solve(double time_limit)
{
    this->time_limit = time_limit;

    if (screen > 0) // 1 or 2
    {
        string name = getSolverName();
        name.resize(35, ' ');
        cout << name << ": ";
    }
    // set timer
    start = clock();

    generateRoot();

    while (!open_list.empty())
    {
        auto curr = selectNode();

        if (terminate(curr)) break;

        clock_t t1;
        if (!curr->is_expanded)  // This is not a back-tracking
        {
            curr->conflict = chooseConflict(*curr);
            curr->is_expanded = true;
            if (screen > 1)
                cout << "	Expand " << *curr << "	on " << *(curr->conflict) << endl;

            assert(!hasHigherPriority(curr->conflict->a1, curr->conflict->a2) and 
                !hasHigherPriority(curr->conflict->a2, curr->conflict->a1) );

            t1 = clock();    
            generateChild(0, curr, curr->conflict->a1, curr->conflict->a2);
            runtime_generate_child += (double)(clock() - t1) / CLOCKS_PER_SEC;

            if (curr->children[0] != nullptr)
                pushNode(curr->children[0]);
        }
        else  // We only generate another child node if back-tracking happens
        {
            if (screen > 1)
                cout << "	Backtrack " << *curr << "	on " << *(curr->conflict) << endl;

            open_list.pop();
            num_backtrack ++;
            t1 = clock();    
            generateChild(1, curr, curr->conflict->a2, curr->conflict->a1);
            runtime_generate_child += (double)(clock() - t1) / CLOCKS_PER_SEC;

            if (curr->children[1] != nullptr)
                pushNode(curr->children[1]);
            curr->clear();
        }
    }  // end of while loop
    return solution_found;
}

PBSNode* PBS2::selectNode()
{
	PBSNode* curr = open_list.top();
    update(curr);
    if (!curr->is_expanded)
    {
        num_HL_expanded++;
        curr->time_expanded = num_HL_expanded;
    }
	if (screen > 1)
		cout << endl << "Select " << *curr << endl;
	return curr;
}

string PBS2::getSolverName() const
{
	return "PBS2 with " + search_engines[0]->getName();
}

// shared_ptr<Conflict> PBS2::chooseConflict(const PBSNode &node) const
// {
//     if (screen == 3)
//         printConflicts(node);
//     if (node.conflicts.empty())
//         return nullptr;

//     return node.pri_conflicts.top();
// }
