#pragma once
#include "common.h"

enum conflict_selection {RANDOM, EARLIEST, CONFLICTS, MCONSTRAINTS, FCONSTRAINTS, WIDTH, SINGLETONS};

struct Constraint
{
    int low = -1;
    int high = -1;
    void set(int _low, int _high)
    {
        low = _low;
        high = _high;
    }
};

std::ostream& operator<<(std::ostream& os, const Constraint& constraint);


struct Conflict
{
    // First constraint is a1<-a2 (a1 has lower priority than a2), and the second is a1->a2
    // Do this for partial expansion
	int a1;
	int a2;
    int priority;
    int max_num_ic;  // Maximum number of implicit constraints between a1->a2 and a2->a2
    explicit Conflict(int a1=-1, int a2=-1, int priority=-1, int max_num_ic=-1): 
        a1(a1), a2(a2), priority(priority), max_num_ic(max_num_ic) {}
};

std::ostream& operator<<(std::ostream& os, const Conflict& conflict);

bool operator < (const Conflict& conflict1, const Conflict& conflict2);
