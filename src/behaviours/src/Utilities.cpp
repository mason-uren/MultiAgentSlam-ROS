//Ethan Warner

#include "Utilities.h"
#include <cmath>
#include <angles/angles.h>

Point whereWasTheCube;

float distance_between_points(Point waypoint, Point currentLocation) {
    return hypot(waypoint.x - currentLocation.x, waypoint.y - currentLocation.y);
}

float angle_between_points(Point waypoint, Point currentLocation) {
    return atan2(waypoint.y - currentLocation.y, waypoint.x - currentLocation.x);
}

float difference_between_angles(Point waypoint, Point currentLocation) {
    return angles::shortest_angular_distance(waypoint.theta, currentLocation.theta);
}

int saturation_check(int direction, int sat) {
    if (direction > sat) { direction = sat; }
    if (direction < -sat) { direction = -sat; }
    return direction;
} //needs to be changed with claudia's function! SWAR-17


float infinity_norm(Point input) {
	return fmaxf(std::abs(input.x), std::abs(input.y));
}


Point GetLastCubeLocation() {
    return whereWasTheCube;
}

void SetLastCubeLocation(float x, float y, float theta) {
    whereWasTheCube.x = x; whereWasTheCube.y = y; whereWasTheCube.theta = theta;
}