#include "Conflict.h"

std::ostream& operator<<(std::ostream& os, const Constraint& constraint)
{
	os << "<" << constraint.low << " is lower than " << constraint.high << ">";
	return os;
}


std::ostream& operator<<(std::ostream& os, const Conflict& conflict)
{
	os << "<" << conflict.a1 << "," << conflict.a2 << ">";
	return os;
}

bool operator < (const Conflict& conflict1, const Conflict& conflict2) // return true if conflict2 has higher priority
{
	return rand() % 2;
}

