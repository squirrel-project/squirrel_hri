#include "squirrel_person_tracker/floor_pointer.h"
#include "gtest/gtest.h"

TEST(FloorPointerTest, testIsGestureReliable)
{
  FloorPointer floorpointer;
  double avg1, avg2, avg3;
  avg1 = avg2 = avg3 = 0;

  EXPECT_FLOAT_EQ(false, floorpointer.isAverage(FloorPointer::NONE));
  EXPECT_FLOAT_EQ(false, floorpointer.isAverage(FloorPointer::LEFT));
  EXPECT_FLOAT_EQ(false, floorpointer.isAverage(FloorPointer::RIGHT));

  for (int i = 0; i < floorpointer.maxQueuesize; ++i)
  {
    nite::Point3f testPoint(i, i + 1, i + 2);
    avg1 += i;
    avg2 += i + 1;
    avg3 += i + 2;
    floorpointer.leftHandPositionHistory.push_back(testPoint);
    floorpointer.rightHandPositionHistory.push_back(testPoint);
  }
  avg1 = avg1 / floorpointer.maxQueuesize;
  avg2 = avg2 / floorpointer.maxQueuesize;
  avg3 = avg3 / floorpointer.maxQueuesize;

  EXPECT_FLOAT_EQ(false, floorpointer.isAverage(FloorPointer::NONE));
  EXPECT_FLOAT_EQ(true, floorpointer.isAverage(FloorPointer::LEFT));
  EXPECT_FLOAT_EQ(true, floorpointer.isAverage(FloorPointer::RIGHT));
  EXPECT_FLOAT_EQ(avg1, floorpointer.average[0]);
  EXPECT_FLOAT_EQ(avg2, floorpointer.average[1]);
  EXPECT_FLOAT_EQ(avg3, floorpointer.average[2]);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
