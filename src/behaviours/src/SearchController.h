#ifndef SEARCH_CONTROLLER
#define SEARCH_CONTROLLER

#include <random_numbers/random_numbers.h>
#include <string>
#include <stack>          // std::stack
#include "Controller.h"
#include "Pose.h"

/**
 * This class implements the search control algorithm for the rovers. The code
 * here should be modified and enhanced to improve search performance.
 */
class SearchController : virtual Controller {

public:

  SearchController();

  void Reset() override;

  // performs search pattern
  Result DoWork() override;
  bool ShouldInterrupt() override;
  bool HasWork() override;

  // sets the value of the current location
  //void UpdateData(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D centerLocation);
  void SetCurrentLocation(Point currentLocation);
  void SetCenterLocation(Point centerLocation);
  void SetSuccesfullPickup();
  void fillStack(float waypoints_x [], float waypoints_y [], int array_length);
  Point getNextWaypoint(Point current_location);
  Point getCurrentWaypoint(Point current_location);
  bool isSearchFinished();
  Point generateRandomWaypoint(Point current_location);
  float generateRandomFloat(float low, float high);

protected:

  void ProcessData();

private:

  random_numbers::RandomNumberGenerator* rng;
  Point currentLocation;
  Point centerLocation;
  Point searchLocation;
  int attemptCount = 0;
  //struct for returning data to ROS adapter
  Result result;

  // Search state
  // Flag to allow special behaviour for the first waypoint
  bool first_waypoint = true;
  bool succesfullPickup = false;
  std::stack<Point> stack_waypoints;
};

#endif /* SEARCH_CONTROLLER */
