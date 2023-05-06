#include "cast.h"

uintptr_t
warthog::cpd::wt_to_label(double& b)
{ return conv<uintptr_t, double>(b); }

double
warthog::cpd::label_to_wt(uintptr_t &b)
{ return conv<double, uintptr_t>(b); }
