// Point.h
class Point
{
public:
	Point(long x, long y);
	Point(long x, long y, long z);
	~Point();
	long GetX();
	long GetY();
	long GetZ();
private:
	long X;
	long Y;
	long Z;
};

