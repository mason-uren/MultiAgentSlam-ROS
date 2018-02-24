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
    result.type = vectorDriving;
    result.desired_heading = GetNewHeading(currentLocation.theta,true); //bool value is search_mode
    printf("search Controller new heading: %f\n",result.desired_heading);
    return result;
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
  std::uniform_real_distribution<float> distribution(-2.355,-3.928);
  float theta = 0;
    theta = currentLocation.theta + (distribution(generator));
  return theta;
}

