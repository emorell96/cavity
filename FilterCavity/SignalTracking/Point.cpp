#include "Point.h"

Point::Point(long x, long y)
{
	X = x;
	Y = y;
	Z = 0;
}

Point::Point(long x, long y, long z) {
	X = x;
	Y = y;
	Z = z;
}

Point::~Point()
{
}

long Point::GetX() 
{
	return X;
}

long Point::GetY() {
	return Y;
}

long Point::GetZ() {
	return Z;
}

