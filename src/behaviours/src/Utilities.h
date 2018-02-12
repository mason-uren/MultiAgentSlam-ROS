//Ethan Warner

#ifndef UTILITIES
#define UTILITIES

#include "Point.h"

class Utilities {

public:
    static float distance_between_points(Point, Point);

    static float angle_between_points(Point, Point);

    static float difference_between_angles(Point, Point);

    static int saturation_check(int, int);
};

#endif
