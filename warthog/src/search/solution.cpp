#include "solution.h"

std::ostream& operator<<(std::ostream& str, warthog::solution& sol)
{
    sol.print(str);
    return str;
}
