// PdhSignalTracker.h
#include "Point.h"

class PdhSignalTracker
{
public:
	PdhSignalTracker();
	~PdhSignalTracker();

	Point Maxima = Point(0, 0);
	Point Minima = Point(0, 0);
	void AddPoint(Point& point);
	void Clear();
	void SetPass(int passnumber);
	
	bool IsSlopeRising();
	 
	long GetSetPoint();
	int GetPassNumber();

private:
	bool firstTime = true;
	int passNumber = 0;
};

