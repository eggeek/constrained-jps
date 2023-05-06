#ifndef __CAST_H_
#define __CAST_H_

#include <cstdint>
#include <cstring>

namespace warthog
{

namespace cpd
{
// Thanks Graeme
//
// Hack of reinterpret_cast
template<class T, class U>
T conv(const U& x)
{
    static_assert(sizeof(T) == sizeof(U),
                  "Should bit-cast between values of equal size");
    T ret;
    memcpy(&ret, &x, sizeof(U));
    return ret;
}

uintptr_t
wt_to_label(double& b);

double
label_to_wt(uintptr_t& b);

} // namespace cpd

} // namespace warthog

#endif // __CAST_H_
