//Ethan Warner

#ifndef UTILITIES
#define UTILITIES

#include "Point.h"

float distance_between_points(Point, Point);

float angle_between_points(Point, Point);

float difference_between_angles(Point, Point);

int saturation_check(int, int);

float infinity_norm(Point);

Point GetLastCubeLocation();

void SetLastCubeLocation(Point lastKnownLocation);

#endif
