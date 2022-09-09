#include "PdhSignalTracker.h"
PdhSignalTracker::PdhSignalTracker()
{
}

PdhSignalTracker::~PdhSignalTracker()
{
}

void PdhSignalTracker::AddPoint(Point& point)
{
	if (firstTime) {
		Maxima = point;
		Minima = point;
		firstTime = false;
	}

	if (point.GetY() > Maxima.GetY()) {
		Maxima = point;
	}
	if (point.GetY() < Minima.GetY()) {
		Minima = point;
	}
}

bool PdhSignalTracker::IsSlopeRising() {
	return (Maxima.GetY() - Minima.GetY()) / (Maxima.GetX() - Minima.GetX()) > 0;
}

// Gets the set point level to use with this PDH.
long PdhSignalTracker::GetSetPoint() {
	return (Maxima.GetY() - Minima.GetY()) / 2;
}

void PdhSignalTracker::SetPass(int number) {
	passNumber = number;
}

int PdhSignalTracker::GetPassNumber() {
	return passNumber;
}

void PdhSignalTracker::Clear() {
	Maxima = Point(0, 0);
	Minima = Point(0, 0);
	firstTime = true;
}
