#include "SearchController.h"
#include <angles/angles.h>

SearchController::SearchController() {
    rng = new random_numbers::RandomNumberGenerator();
    currentLocation.x = 0;
    currentLocation.y = 0;
    currentLocation.theta = 0;

    // centerLocation.x = 0; 
    // centerLocation.y = 0;
    // centerLocation.theta = 0;
    result.PIDMode = FAST_PID;

    result.fingerAngle = M_PI / 2;
    result.wristAngle = M_PI / 4;

    /*
     * TODO:
     */
    this->controller = SEARCH;
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
    Point searchLocation = GetLastCubeLocation();
    cout << "waypoint outside wall timer: " << waypoint_outside_wall_timer << endl;
    cout << "waypoint search timer start: " << waypoint_search_timer_start << endl;
    waypoint_outside_wall_timer = (current_time - waypoint_search_timer_start) / 1e3;
    if( searchLocation.x != 0 && searchLocation.y != 0 && abandonShip == false) {
        cout << "searhccontroller has a known location: " << searchLocation.x << searchLocation.y << endl;
        result.type = waypoint;
        result.waypoints.clear();
        result.waypoints.insert(result.waypoints.begin(), searchLocation);
        cout << "inside if waypoint outside wall timer: " << waypoint_outside_wall_timer << endl;
        if(waypoint_outside_wall_timer > 120)//2 minutes, could be longer
        {
            abandonShip = true;
            cout << "Abadoning going to last cube location, returning to vector driving" << endl;
            result.type = vectorDriving;
        }
    } else {
        result.type = vectorDriving;
    }
    /*if( waypoint_outside_wall_timer > 120){
        result.type = vectorDriving;
    }*/
    return result;

}

void SearchController::SetCenterLocation(Point centerLocation) {

    // float diffX = this->centerLocation.x - centerLocation.x;
    // float diffY = this->centerLocation.y - centerLocation.y;
    // this->centerLocation = centerLocation;

    // if (!result.waypoints.empty()) {
    //     result.waypoints.back().x -= diffX;
    //     result.waypoints.back().y -= diffY;
    // }

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
    waypoint_search_timer_start = current_time;
    cout << "Starting waypoint search timer:  " << waypoint_search_timer_start << endl;//Move this so it starts the timer after a good dropoff
    succesfullPickup = true;
    abandonShip = false;
}

float SearchController::GetNewHeading(float beta, bool search_mode) {
  std::default_random_engine generator;
  std::uniform_real_distribution<float> distribution(-2.355,-3.928);
  float theta = 0;
    theta = currentLocation.theta;// + (distribution(generator));
  return theta;
}

