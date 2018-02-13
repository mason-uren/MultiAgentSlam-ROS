//Ethan Warner

#include "Utilities.h"
#include <cmath>
#include <angles/angles.h>


float Utilities::distance_between_points(Point waypoint, Point currentLocation) {
    return hypot(waypoint.x - currentLocation.x, waypoint.y - currentLocation.y);
}

float Utilities::angle_between_points(Point waypoint, Point currentLocation) {
    return atan2(waypoint.y - currentLocation.y, waypoint.x - currentLocation.x);
}

float Utilities::difference_between_angles(Point waypoint, Point currentLocation) {
    return angles::shortest_angular_distance(waypoint.theta, currentLocation.theta);
}

int Utilities::saturation_check(int direction, int sat) {
    if (direction > sat) { direction = sat; }
    if (direction < -sat) { direction = -sat; }
    return direction;
} //needs to be changed with claudia's function! SWAR-17


float Utilities::infinity_norm(Point input) {
	return fmax(std::abs(input.x), std::abs(input.y));
}