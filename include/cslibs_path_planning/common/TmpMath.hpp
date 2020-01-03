
#ifndef TMP_MATH_HPP
#define TMP_MATH_HPP

namespace tmpmath
{
// Compile time rounding to nearest whole number
constexpr int round(double x)
{
    return (x >= 0 ? x + 0.5 : x - 0.5);
}
} // namespace tmpmath

#endif // TMP_MATH_HPP