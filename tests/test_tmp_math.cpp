#include "gtest/gtest.h"

#include "cslibs_path_planning/common/TmpMath.hpp"

#include <cmath>

void testRound(const double value)
{
    EXPECT_EQ(tmpmath::round(value), std::round(value))
        << "tmpmath::round(" << value << ") != " << std::round(value);
}

TEST(TmpMath, RoundPositive)
{
    testRound(-1.5 - std::numeric_limits<double>::epsilon());
    testRound(-1.5);
    testRound(-1.5 + std::numeric_limits<double>::epsilon());
    testRound(-1.2);
    testRound(-1.0);
    testRound(-0.5 - std::numeric_limits<double>::epsilon());
    testRound(-0.5);
    testRound(-0.5 + std::numeric_limits<double>::epsilon());
    testRound(-0.0);
    testRound(0.0);
    testRound(0.5 - std::numeric_limits<double>::epsilon());
    testRound(0.5);
    testRound(0.5 + std::numeric_limits<double>::epsilon());
    testRound(1.0);
    testRound(1.2);
    testRound(1.49);
    testRound(1.5 - std::numeric_limits<double>::epsilon());
    testRound(1.5);
    testRound(1.5 + std::numeric_limits<double>::epsilon());
}