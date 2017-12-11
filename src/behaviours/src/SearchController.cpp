#include "SearchController.h"
#include <angles/angles.h>


SearchController::SearchController() {
  rng = new random_numbers::RandomNumberGenerator();
  currentLocation.x = 0;
  currentLocation.y = 0;
  currentLocation.theta = 0;

  centerLocation.x = 0;
  centerLocation.y = 0;
  centerLocation.theta = 0;
  result.PIDMode = FAST_PID;

  result.fingerAngle = M_PI/2;
  result.wristAngle = M_PI/4;
  float waypoints_x1 [] = {2.5, -2.5, -2.5, 2.5, 2.5, -2.5, 0.0};
  float waypoints_y1 [] = {2.5, 2.5, -2.5, -2.5, 2.5, 2.5, 0.0};
  int array_length = sizeof(waypoints_x1)/sizeof(waypoints_x1[0]);
  fillStack(waypoints_x1, waypoints_y1, array_length);
}

void SearchController::fillStack(float waypoints_x [], float waypoints_y [], int array_length)
{
    Point next_position;
    for (int i = array_length-1; i >= 0; i--)
    {
        next_position.x = waypoints_x[i];
        next_position.y = waypoints_y[i];
        stack_waypoints.push(next_position);
    }
}

Point SearchController::getNextWaypoint(Point current_location)
{
    Point next_waypoint;
        if(stack_waypoints.empty())
        {
            next_waypoint = generateRandomWaypoint(current_location);
            stack_waypoints.push(next_waypoint);
        }
        else
        {
            stack_waypoints.pop();
            if(stack_waypoints.empty())
            {
                next_waypoint = generateRandomWaypoint(current_location);
                stack_waypoints.push(next_waypoint);
            }
            else
            {
                next_waypoint = stack_waypoints.top();
            }
        }
    return next_waypoint;
}

Point SearchController::getCurrentWaypoint(Point current_location)
{
    Point current_waypoint;
    if(stack_waypoints.empty())
    {
         current_waypoint = generateRandomWaypoint(current_location);
    }
    else
    {
        current_waypoint = stack_waypoints.top();
    }
    return current_waypoint;
}

bool SearchController::isSearchFinished()
{
    return stack_waypoints.size()<=1;
}

Point SearchController::generateRandomWaypoint(Point current_location)
{
    Point next_waypoint;
    double new_theta = generateRandomFloat(current_location.theta-0.25,current_location.theta+0.25);
    double new_radius = generateRandomFloat(0,2.0);
    next_waypoint.x = current_location.x + new_radius * cos(new_theta);
    next_waypoint.y = current_location.y + new_radius * sin(new_theta);
    return next_waypoint;
}

float SearchController::generateRandomFloat(float low, float high)
{
    float random_float = low + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(high-low)));
    return random_float;
}

void SearchController::Reset() {
  result.reset = false;
}

/**
 * This code implements a basic random walk search.
 */
Result SearchController::DoWork() {

  if (!result.wpts.waypoints.empty()) {
    if (hypot(result.wpts.waypoints[0].x-currentLocation.x, result.wpts.waypoints[0].y-currentLocation.y) < 0.15) {
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
  }
  else if (attemptCount >= 5 || attemptCount == 0) 
  {
    attemptCount = 1;


    result.type = waypoint;
    Point  searchLocation;

//    //select new position 50 cm from current location
//    if (first_waypoint)
//    {
//      first_waypoint = false;
//      searchLocation.theta = currentLocation.theta + M_PI;
//      searchLocation.x = currentLocation.x + (0.5 * cos(searchLocation.theta));
//      searchLocation.y = currentLocation.y + (0.5 * sin(searchLocation.theta));
//    }
//    else
//    {
//      //select new heading from Gaussian distribution around current heading
//      searchLocation.theta = rng->gaussian(currentLocation.theta, 0.785398); //45 degrees in radians
//      searchLocation.x = currentLocation.x + (0.5 * cos(searchLocation.theta));
//      searchLocation.y = currentLocation.y + (0.5 * sin(searchLocation.theta));
//    }
    searchLocation = getNextWaypoint(currentLocation);

    result.wpts.waypoints.clear();
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);
    
    return result;
  }

}

void SearchController::SetCenterLocation(Point centerLocation) {
  
  float diffX = this->centerLocation.x - centerLocation.x;
  float diffY = this->centerLocation.y - centerLocation.y;
  this->centerLocation = centerLocation;
  
  if (!result.wpts.waypoints.empty())
  {
  result.wpts.waypoints.back().x -= diffX;
  result.wpts.waypoints.back().y -= diffY;
  }
  
}

void SearchController::SetCurrentLocation(Point currentLocation) {
  this->currentLocation = currentLocation;
}

void SearchController::ProcessData() {
}

bool SearchController::ShouldInterrupt(){
  ProcessData();

  return false;
}

bool SearchController::HasWork() {
  return true;
}

void SearchController::SetSuccesfullPickup() {
  succesfullPickup = true;
}


