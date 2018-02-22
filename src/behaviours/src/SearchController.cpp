#include "SearchController.h"
#include <angles/angles.h>
#include "Utilities.h"

SearchController::SearchController() {
    rng = new random_numbers::RandomNumberGenerator();
    currentLocation.x = 0;
    currentLocation.y = 0;
    currentLocation.theta = 0;

    centerLocation.x = 0;
    centerLocation.y = 0;
    centerLocation.theta = 0;
    result.PIDMode = FAST_PID;

    result.fingerAngle = M_PI / 2;
    result.wristAngle = M_PI / 4;
}

void SearchController::Reset() {
    result.reset = false;
}

void SearchController::SetCurrentTimeInMilliSecs(long int time) {
    current_time = time;
}


/**
 * This code implements a basic random walk search.
 */

Result SearchController::DoWork() {

    extern void logicMessage(long int currentTime, string component, string message);
    if (vector_search) {
        printf("search ctrller");
        result.type = vectorDriving;
        result.desired_heading = GetNewHeading(currentLocation.theta,true); //bool value is search_mode
        printf(" %f\n",result.desired_heading);
        return result;
    } else {
        if (!result.waypoints.empty()) {
            if (Utilities::distance_between_points(result.waypoints[0], currentLocation) < 0.15) {
                attemptCount = 0;
            }
        }
        if (attemptCount > 0 && attemptCount < 5) {
            attemptCount++;
            if (succesfullPickup) {
                succesfullPickup = false;
                attemptCount = 1;
            }
            return result;
        } else if (attemptCount >= 5 || attemptCount == 0) {
            attemptCount = 1;


            result.type = waypoint;
            Point searchLocation;

            //select new position 50 cm from current location
            if (first_waypoint) {
                first_waypoint = false;
                searchLocation.theta = currentLocation.theta + M_PI;
                searchLocation.x = currentLocation.x + (0.5 * cos(searchLocation.theta));
                searchLocation.y = currentLocation.y + (0.5 * sin(searchLocation.theta));
            } else {
                //select new heading from Gaussian distribution around current heading
                searchLocation.theta = rng->gaussian(currentLocation.theta, 0.785398); //45 degrees in radians
                searchLocation.x = currentLocation.x + (0.5 * cos(searchLocation.theta));
                searchLocation.y = currentLocation.y + (0.5 * sin(searchLocation.theta));
            }

            result.waypoints.clear();
            result.waypoints.insert(result.waypoints.begin(), searchLocation);
        }
    return result;
    }
}

void SearchController::SetCenterLocation(Point centerLocation) {

    float diffX = this->centerLocation.x - centerLocation.x;
    float diffY = this->centerLocation.y - centerLocation.y;
    this->centerLocation = centerLocation;

    if (!result.waypoints.empty()) {
        result.waypoints.back().x -= diffX;
        result.waypoints.back().y -= diffY;
    }

}

void SearchController::SetCurrentLocation(Point currentLocation) {
    this->currentLocation = currentLocation;
}

void SearchController::ProcessData() {
}

bool SearchController::ShouldInterrupt() {
    ProcessData();

    return false;
}

bool SearchController::HasWork() {
    return true;
}

void SearchController::SetSuccesfullPickup() {
    succesfullPickup = true;
}

float SearchController::GetNewHeading(float beta, bool search_mode) {
  std::default_random_engine generator;
  std::uniform_real_distribution<float> distribution(0.0,M_PI);
  float theta = 0;
  if(search_mode == false) {
    theta = beta - M_PI/2 + (distribution(generator));
  }
  else {
    theta = beta - M_PI/2 - (distribution(generator));
  }
  return theta;
}

