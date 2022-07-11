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
	int a1;
	int a2;
    explicit Conflict(int a1 = -1, int a2 = -1): a1(a1), a2(a2) { }
};

std::ostream& operator<<(std::ostream& os, const Conflict& conflict);

bool operator < (const Conflict& conflict1, const Conflict& conflict2);
