#include "csv.h"
#include "pch.h"
#include <PdhSignalTracker.h>

TEST(TestCaseName, TestName) {
  EXPECT_EQ(1, 1);
  EXPECT_TRUE(true);
}

TEST(TestPdhSignalTracking, FindsMinimaAndMaxima) {
	PdhSignalTracker tracker;

	io::CSVReader<1> in("C:\\Users\\emore\\Documents\\Bouwmeester Lab\\Arduino\\cavity\\FilterCavity\\CppPoundDreverHallTests\\TestData.csv");
	in.read_header(io::ignore_extra_column, "Ydigital");
	long yvalue;

	int i = 0;
	while (in.read_row(yvalue)) {
		// do stuff with the data
		tracker.AddPoint(Point(i, yvalue));
		++i;
	}
	EXPECT_EQ(tracker.Maxima.GetX(), 204);
	EXPECT_EQ(tracker.Maxima.GetY(), 19293);

	EXPECT_EQ(tracker.Minima.GetX(), 195);
	EXPECT_EQ(tracker.Minima.GetY(), 7742);

	EXPECT_EQ(tracker.IsSlopeRising(), true);
}