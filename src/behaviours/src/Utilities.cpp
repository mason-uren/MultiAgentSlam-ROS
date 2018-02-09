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

int Utilities::saturation_check_left(int left, int sat) {
    if (left > sat) { left = sat; }
    if (left < -sat) { left = -sat; }
    return left;
}

int Utilities::saturation_check_right(int right, int sat) {
    if (right > sat) { right = sat; }
    if (right < -sat) { right = -sat; }
    return right;
}

