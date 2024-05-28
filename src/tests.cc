#include <gtest/gtest.h>
#include <math.h>
#ifdef __cplusplus
extern "C" {
    #include "geom_utils.h"
}
#endif

TEST(GeometryUtilsTest, TestPointToLineDist)
{
    // Expect two strings not to be equal.
    Vector2 origin = {0, 0};
    Vector2 xAxis = {3, 0};
    Vector2 yAxis = {0, 10};
    Vector2 pt1 = {1, 1};
    float dist1 = pointToLineDistance(origin, xAxis, pt1, NULL);
    EXPECT_FLOAT_EQ(dist1, 1.0f);
    float dist2 = pointToLineDistance(origin, yAxis, pt1, NULL);
    EXPECT_FLOAT_EQ(dist2, 1.0f);
}