#include "SearchController.h"

SearchController::SearchController() {
  rng = new random_numbers::RandomNumberGenerator();
}

void SearchController::setStack(std::string botName) {
  // Start: From old code:
  const float d = 0.5;
  const double final_boundary = 11.0;
  float waypoints_x_final [] {final_boundary-d, -(final_boundary-d), -(final_boundary-d),    final_boundary-d,   final_boundary-d, -(final_boundary-2*d), -(final_boundary-2*d),    final_boundary-2*d, final_boundary-2*d, -(final_boundary-3*d), -(final_boundary-3*d),    final_boundary-3*d, final_boundary-3*d, -(final_boundary-4*d), -(final_boundary-4*d),   final_boundary-4*d, final_boundary-4*d};
  float waypoints_y_final [] {final_boundary-d,    final_boundary-d, -(final_boundary-d), -(final_boundary-d), final_boundary-2*d,    final_boundary-2*d, -(final_boundary-2*d), -(final_boundary-2*d), final_boundary-3*d,    final_boundary-3*d, -(final_boundary-3*d), -(final_boundary-3*d), final_boundary-4*d,    final_boundary-4*d, -(final_boundary-4*d), -(final_boundary-4*d), final_boundary-5*d};

  float waypoints_x2_final [] {final_boundary-4*d, -(final_boundary-5*d), -(final_boundary-5*d),    final_boundary-5*d,    final_boundary-5*d, -(final_boundary-6*d), -(final_boundary-6*d),    final_boundary-6*d, final_boundary-6*d,  -(final_boundary-7*d), -(final_boundary-7*d),    final_boundary-7*d, final_boundary-7*d, -(final_boundary-8*d), -(final_boundary-8*d),    final_boundary-8*d, final_boundary-8*d, final_boundary-8*d, -(final_boundary-9*d), -(final_boundary-9*d),   final_boundary-9*d};
  float waypoints_y2_final [] {final_boundary-5*d,    final_boundary-5*d, -(final_boundary-5*d), -(final_boundary-5*d),    final_boundary-6*d,    final_boundary-6*d, -(final_boundary-6*d), -(final_boundary-6*d), final_boundary-7*d,     final_boundary-7*d, -(final_boundary-7*d), -(final_boundary-7*d), final_boundary-8*d,    final_boundary-8*d, -(final_boundary-8*d), -(final_boundary-8*d), final_boundary-9*d, final_boundary-9*d,    final_boundary-9*d, -(final_boundary-9*d), -(final_boundary-9*d)};

  float waypoints_x3_final [] { final_boundary-9*d,  -(final_boundary-10*d), -(final_boundary-10*d),   final_boundary-10*d,  final_boundary-10*d, -(final_boundary-11*d), -(final_boundary-11*d),    final_boundary-11*d, final_boundary-11*d, -(final_boundary-12*d), -(final_boundary-12*d),    final_boundary-12*d, final_boundary-12*d, -(final_boundary-13*d), -(final_boundary-13*d),    final_boundary-13*d, final_boundary-13*d, -(final_boundary-14*d), -(final_boundary-14*d),    final_boundary-14*d, final_boundary-14*d, -(final_boundary-15*d), -(final_boundary-15*d),    final_boundary-15*d, final_boundary-15*d, -(final_boundary-16*d), -(final_boundary-16*d),    final_boundary-16*d, final_boundary-16*d,  -(final_boundary-17*d), -(final_boundary-17*d),    final_boundary-17*d, final_boundary-17*d, -(final_boundary-18*d), -(final_boundary-18*d),    final_boundary-18*d, final_boundary-18*d,  -(final_boundary-19*d), -(final_boundary-19*d),    final_boundary-19*d, final_boundary-19*d};
  float waypoints_y3_final [] {final_boundary-10*d,     final_boundary-10*d, -(final_boundary-10*d), -(final_boundary-10*d), final_boundary-11*d,    final_boundary-11*d, -(final_boundary-11*d), -(final_boundary-11*d), final_boundary-12*d,    final_boundary-12*d, -(final_boundary-12*d), -(final_boundary-12*d), final_boundary-13*d,    final_boundary-13*d, -(final_boundary-13*d), -(final_boundary-13*d), final_boundary-14*d,    final_boundary-14*d, -(final_boundary-14*d), -(final_boundary-14*d), final_boundary-15*d,    final_boundary-15*d, -(final_boundary-15*d), -(final_boundary-15*d), final_boundary-16*d,    final_boundary-16*d, -(final_boundary-16*d), -(final_boundary-16*d), final_boundary-17*d,     final_boundary-17*d, -(final_boundary-17*d), -(final_boundary-17*d), final_boundary-18*d,    final_boundary-18*d, -(final_boundary-18*d), -(final_boundary-18*d), final_boundary-19*d,     final_boundary-19*d, -(final_boundary-19*d), -(final_boundary-19*d), final_boundary-20*d};

  const double preliminary_boundary = 7.5;
//  float waypoints_x_preliminary [] {preliminary_boundary-d, -(preliminary_boundary-d), -(preliminary_boundary-d),    preliminary_boundary-d,   preliminary_boundary-d, -(preliminary_boundary-2*d), -(preliminary_boundary-2*d),    preliminary_boundary-2*d, preliminary_boundary-2*d, -(preliminary_boundary-3*d), -(preliminary_boundary-3*d),    preliminary_boundary-3*d, preliminary_boundary-3*d, -(preliminary_boundary-4*d), -(preliminary_boundary-4*d),   preliminary_boundary-4*d, preliminary_boundary-4*d,  -(preliminary_boundary-5*d), -(preliminary_boundary-5*d),    preliminary_boundary-5*d,    preliminary_boundary-5*d};
//  float waypoints_y_preliminary [] {preliminary_boundary-d,    preliminary_boundary-d, -(preliminary_boundary-d), -(preliminary_boundary-d), preliminary_boundary-2*d,    preliminary_boundary-2*d, -(preliminary_boundary-2*d), -(preliminary_boundary-2*d), preliminary_boundary-3*d,    preliminary_boundary-3*d, -(preliminary_boundary-3*d), -(preliminary_boundary-3*d), preliminary_boundary-4*d,    preliminary_boundary-4*d, -(preliminary_boundary-4*d), -(preliminary_boundary-4*d), preliminary_boundary-5*d,    preliminary_boundary-5*d, -(preliminary_boundary-5*d), -(preliminary_boundary-5*d),    preliminary_boundary-6*d};

//  float waypoints_x2_preliminary [] {preliminary_boundary-5*d, -(preliminary_boundary-6*d), -(preliminary_boundary-6*d),    preliminary_boundary-6*d, preliminary_boundary-6*d, -(preliminary_boundary-7*d), -(preliminary_boundary-7*d),    preliminary_boundary-7*d, preliminary_boundary-7*d, -(preliminary_boundary-8*d), -(preliminary_boundary-8*d),    preliminary_boundary-8*d, preliminary_boundary-8*d, -(preliminary_boundary-9*d), -(preliminary_boundary-9*d),   preliminary_boundary-9*d,   preliminary_boundary-9*d,  -(preliminary_boundary-10*d), -(preliminary_boundary-10*d),   preliminary_boundary-10*d,  preliminary_boundary-10*d, -(preliminary_boundary-11*d), -(preliminary_boundary-11*d),    preliminary_boundary-11*d, preliminary_boundary-11*d, -(preliminary_boundary-12*d), -(preliminary_boundary-12*d),    preliminary_boundary-12*d, preliminary_boundary-12*d};
//  float waypoints_y2_preliminary [] {preliminary_boundary-6*d,    preliminary_boundary-6*d, -(preliminary_boundary-6*d), -(preliminary_boundary-6*d), preliminary_boundary-7*d,    preliminary_boundary-7*d, -(preliminary_boundary-7*d), -(preliminary_boundary-7*d), preliminary_boundary-8*d,    preliminary_boundary-8*d, -(preliminary_boundary-8*d), -(preliminary_boundary-8*d), preliminary_boundary-9*d,    preliminary_boundary-9*d, -(preliminary_boundary-9*d), -(preliminary_boundary-9*d), preliminary_boundary-10*d,     preliminary_boundary-10*d, -(preliminary_boundary-10*d), -(preliminary_boundary-10*d), preliminary_boundary-11*d,    preliminary_boundary-11*d, -(preliminary_boundary-11*d), -(preliminary_boundary-11*d), preliminary_boundary-12*d,    preliminary_boundary-12*d, -(preliminary_boundary-12*d), -(preliminary_boundary-12*d), preliminary_boundary-13*d};


  float waypoints_x2_preliminary [] {preliminary_boundary-d, -(preliminary_boundary-2*d), -(preliminary_boundary-2*d),    preliminary_boundary-2*d, preliminary_boundary-2*d, -(preliminary_boundary-3*d), -(preliminary_boundary-3*d),    preliminary_boundary-3*d, preliminary_boundary-3*d, -(preliminary_boundary-4*d), -(preliminary_boundary-4*d),   preliminary_boundary-4*d, preliminary_boundary-4*d};
  float waypoints_y2_preliminary [] {preliminary_boundary-2*d,    preliminary_boundary-2*d, -(preliminary_boundary-2*d), -(preliminary_boundary-2*d), preliminary_boundary-3*d,    preliminary_boundary-3*d, -(preliminary_boundary-3*d), -(preliminary_boundary-3*d), preliminary_boundary-4*d,    preliminary_boundary-4*d, -(preliminary_boundary-4*d), -(preliminary_boundary-4*d), preliminary_boundary-5*d};

//  float waypoints_x_preliminary [] {preliminary_boundary-3*d, -(preliminary_boundary-4*d), -(preliminary_boundary-4*d),   preliminary_boundary-4*d, preliminary_boundary-4*d};
//  float waypoints_y_preliminary [] {preliminary_boundary-4*d,    preliminary_boundary-4*d, -(preliminary_boundary-4*d), -(preliminary_boundary-4*d), preliminary_boundary-5*d};

  float waypoints_x3_preliminary [] {preliminary_boundary-4*d, -(preliminary_boundary-5*d), -(preliminary_boundary-5*d),    preliminary_boundary-5*d,    preliminary_boundary-5*d, -(preliminary_boundary-6*d), -(preliminary_boundary-6*d),    preliminary_boundary-6*d, preliminary_boundary-6*d,  -(preliminary_boundary-7*d), -(preliminary_boundary-7*d),    preliminary_boundary-7*d, preliminary_boundary-7*d, -(preliminary_boundary-8*d), -(preliminary_boundary-8*d),    preliminary_boundary-8*d, preliminary_boundary-8*d, preliminary_boundary-8*d, -(preliminary_boundary-9*d), -(preliminary_boundary-9*d),   preliminary_boundary-9*d};
  float waypoints_y3_preliminary [] {preliminary_boundary-5*d,    preliminary_boundary-5*d, -(preliminary_boundary-5*d), -(preliminary_boundary-5*d),    preliminary_boundary-6*d,    preliminary_boundary-6*d, -(preliminary_boundary-6*d), -(preliminary_boundary-6*d), preliminary_boundary-7*d,     preliminary_boundary-7*d, -(preliminary_boundary-7*d), -(preliminary_boundary-7*d), preliminary_boundary-8*d,    preliminary_boundary-8*d, -(preliminary_boundary-8*d), -(preliminary_boundary-8*d), preliminary_boundary-9*d, preliminary_boundary-9*d,    preliminary_boundary-9*d, -(preliminary_boundary-9*d), -(preliminary_boundary-9*d)};

  float waypoints_x_preliminary [] { preliminary_boundary-9*d,  -(preliminary_boundary-10*d), -(preliminary_boundary-10*d),   preliminary_boundary-10*d,  preliminary_boundary-10*d, -(preliminary_boundary-11*d), -(preliminary_boundary-11*d),    preliminary_boundary-11*d, preliminary_boundary-11*d, -(preliminary_boundary-12*d), -(preliminary_boundary-12*d),    preliminary_boundary-12*d, preliminary_boundary-12*d, -(preliminary_boundary-13*d), -(preliminary_boundary-13*d),    preliminary_boundary-13*d, preliminary_boundary-13*d, -(preliminary_boundary-14*d), -(preliminary_boundary-14*d),    preliminary_boundary-14*d};
  float waypoints_y_preliminary [] {preliminary_boundary-10*d,     preliminary_boundary-10*d, -(preliminary_boundary-10*d), -(preliminary_boundary-10*d), preliminary_boundary-11*d,    preliminary_boundary-11*d, -(preliminary_boundary-11*d), -(preliminary_boundary-11*d), preliminary_boundary-12*d,    preliminary_boundary-12*d, -(preliminary_boundary-12*d), -(preliminary_boundary-12*d), preliminary_boundary-13*d,    preliminary_boundary-13*d, -(preliminary_boundary-13*d), -(preliminary_boundary-13*d), preliminary_boundary-14*d,    preliminary_boundary-14*d, -(preliminary_boundary-14*d), -(preliminary_boundary-14*d)};


  // End: From old code:



  //stacks to store waypoints

  geometry_msgs::Pose2D nextPosition;
  bool finalRound = false; // We need to fix this later.  Right now I just want to get prelim working.
  //push waypoints onto stack
  if (finalRound)
  {
      if(botName == "ajax")
      {
          for (int i = sizeof(waypoints_x_final)/sizeof(waypoints_x_final[0]); i > 0; i--)
          {
              nextPosition.x = waypoints_x_final[i];
              nextPosition.y = waypoints_y_final[i];
              stack_waypoints.push(nextPosition);
          }
      }
      else if(botName == "achilles")
      {
          for (int i = sizeof(waypoints_x2_final)/sizeof(waypoints_x2_final[0]); i > 0; i--)
          {
              nextPosition.x = waypoints_x2_final[i];
              nextPosition.y = waypoints_y2_final[i];
              stack_waypoints.push(nextPosition);
          }
      }
      else if(botName == "aeneas")
      {
          for (int i = sizeof(waypoints_x3_final)/sizeof(waypoints_x3_final[0]); i > 0; i--)
          {
              nextPosition.x = waypoints_x3_final[i];
              nextPosition.y = waypoints_y3_final[i];
              stack_waypoints.push(nextPosition);
          }
      }
  }
  else
  {
      if(botName == "ajax")
      {
//          for (int i = sizeof(waypoints_x_preliminary)/sizeof(waypoints_x_preliminary[0]); i > 0; i--) // spiral in
          for (int i = 0; i < sizeof(waypoints_x_preliminary)/sizeof(waypoints_x_preliminary[0]); i++) // spiral out
          {
              nextPosition.x = waypoints_x_preliminary[i];
              nextPosition.y = waypoints_y_preliminary[i];
              stack_waypoints.push(nextPosition);
          }
      }
      else if(botName == "achilles")
      {
//          for (int i = sizeof(waypoints_x2_preliminary)/sizeof(waypoints_x2_preliminary[0]); i > 0; i--) // spiral in
          for (int i = 0; i < sizeof(waypoints_x2_preliminary)/sizeof(waypoints_x2_preliminary[0]); i++) // spiral out
          {
              nextPosition.x = waypoints_x2_preliminary[i];
              nextPosition.y = waypoints_y2_preliminary[i];
              stack_waypoints.push(nextPosition);
          }
      }
      else if(botName == "aeneas")
      {
//          for (int i = sizeof(waypoints_x3_preliminary)/sizeof(waypoints_x3_preliminary[0]); i > 0; i--) // spiral in
          for (int i = 0; i < sizeof(waypoints_x3_preliminary)/sizeof(waypoints_x3_preliminary[0]); i++) // spiral out
          {
              nextPosition.x = waypoints_x3_preliminary[i];
              nextPosition.y = waypoints_y3_preliminary[i];
              stack_waypoints.push(nextPosition);
          }
      }
  }
}
///**
// * This code implements a basic random walk search.
// */
//geometry_msgs::Pose2D SearchController::search(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D oldGoalLocation, std::string botName) {

//  geometry_msgs::Pose2D newGoalLocation;

//  //select new heading from Gaussian distribution around current heading
//  // newGoalLocation.theta = rng->gaussian(currentLocation.theta, 0.25);

//  //select new position 50 cm from current location
//  //newGoalLocation.x = currentLocation.x + (0.5 * cos(goalLocation.theta));
//  //newGoalLocation.y = currentLocation.y + (0.5 * sin(goalLocation.theta));

//  // Nick's code to disable movement for testing the gripper.
//  //newGoalLocation.theta = 0.0;
//  //newGoalLocation.x = 0.0;
//  //newGoalLocation.y = 0.0;

//  // This should check to see if we have reached our current waypoint and then call waypointNextLocation if we have.
//  double remainingGoalDist = hypot(oldGoalLocation.x - currentLocation.x, oldGoalLocation.y - currentLocation.y);
//  double thresholdRadius = 0.1; // Units of meters.  This is the maximum distance we can be from the waypoint before calling it good.
//  if(remainGoalDist<thresholdRadius)
//  {
//    newGoalLocation = waypointNextLocation(botName);
//  }

//  return newGoalLocation;
//}

///**
// * Continues search pattern after interruption. For example, avoiding the
// * center or collisions.
// */
//geometry_msgs::Pose2D SearchController::continueInterruptedSearch(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D oldGoalLocation) {
//  geometry_msgs::Pose2D newGoalLocation;

//  //for testing purposes!

//  newGoalLocation.theta = 0.0;
//  newGoalLocation.x = 0.0;
//  newGoalLocation.y = 0.0;
//  //testing...

//  //remainingGoalDist avoids magic numbers by calculating the dist
//  double remainingGoalDist = hypot(oldGoalLocation.x - currentLocation.x, oldGoalLocation.y - currentLocation.y);

//  //this of course assumes random walk continuation. Change for diffrent search methods.
//  //newGoalLocation.theta = oldGoalLocation.theta;
//  //newGoalLocation.x = currentLocation.x + (0.50 * cos(oldGoalLocation.theta)); //(remainingGoalDist * cos(oldGoalLocation.theta));
//  //newGoalLocation.y = currentLocation.y + (0.50 * sin(oldGoalLocation.theta)); //(remainingGoalDist * sin(oldGoalLocation.theta));

//  return newGoalLocation;
//}

/**
 * This code implements a basic random walk search.
 */
//geometry_msgs::Pose2D SearchController::search(geometry_msgs::Pose2D currentLocation) {
//  geometry_msgs::Pose2D goalLocation;

////  //select new heading from Gaussian distribution around current heading
////  goalLocation.theta = rng->gaussian(currentLocation.theta, 0.25);

////  //select new position 50 cm from current location
////  goalLocation.x = currentLocation.x + (0.5 * cos(goalLocation.theta));
////  goalLocation.y = currentLocation.y + (0.5 * sin(goalLocation.theta));

//  goalLocation.x = 5.0;
//  goalLocation.y = 5.0;
//  goalLocation.theta = 1.0;
//  return goalLocation;
//}

/**
 * Continues search pattern after interruption. For example, avoiding the
 * center or collisions.
 */
geometry_msgs::Pose2D SearchController::continueInterruptedSearch(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D oldGoalLocation) {
  geometry_msgs::Pose2D newGoalLocation;

  //remainingGoalDist avoids magic numbers by calculating the dist
  double remainingGoalDist = hypot(oldGoalLocation.x - currentLocation.x, oldGoalLocation.y - currentLocation.y);

  //this of course assumes random walk continuation. Change for diffrent search methods.
  newGoalLocation.theta = oldGoalLocation.theta;
  newGoalLocation.x = currentLocation.x + (0.50 * cos(oldGoalLocation.theta)); //(remainingGoalDist * cos(oldGoalLocation.theta));
  newGoalLocation.y = currentLocation.y + (0.50 * sin(oldGoalLocation.theta)); //(remainingGoalDist * sin(oldGoalLocation.theta));

  return newGoalLocation;
}

void SearchController::waypointSearchFound(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D oldGoalLocation, std::string botName)
{
    stack_waypoints.push(oldGoalLocation);
    stack_waypoints.push(currentLocation);
}

void SearchController::pushWaypoint(geometry_msgs::Pose2D newLocation)
{
    stack_waypoints.push(newLocation);
}

geometry_msgs::Pose2D SearchController::popWaypoint()
{
    geometry_msgs::Pose2D nextWaypoint;
    if(stack_waypoints.empty())
    {
        double newTheta = rng->uniformReal(0, 2 * M_PI); // theta between 0 and 2pi
        double newRadius = rng->uniformReal(0,5); // radius between 0 and 1
        //random new waypoint
        nextWaypoint.x = (newRadius * cos(newTheta)); //(remainingGoalDist * cos(oldGoalLocation.theta));
        nextWaypoint.y = (newRadius * sin(newTheta)); //(remainingGoalDist * sin(oldGoalLocation.theta));
        pushWaypoint(nextWaypoint);
    }
    else
    {
        //push new waypoint from stack
        nextWaypoint = stack_waypoints.top();
        stack_waypoints.pop();
    }
    return nextWaypoint;
}

geometry_msgs::Pose2D SearchController::peekWaypoint()
{
    geometry_msgs::Pose2D nextWaypoint;
    if(stack_waypoints.empty())
    {
        double newTheta = rng->uniformReal(0, 2 * M_PI); // theta between 0 and 2pi
        double newRadius = rng->uniformReal(0,5.0); // radius between 0 and 5 meters
        //random new waypoint
        nextWaypoint.x = (newRadius * cos(newTheta)); //(remainingGoalDist * cos(oldGoalLocation.theta));
        nextWaypoint.y = (newRadius * sin(newTheta)); //(remainingGoalDist * sin(oldGoalLocation.theta));
        pushWaypoint(nextWaypoint);
    }
    else
    {
        //push new waypoint from stack
        nextWaypoint = stack_waypoints.top();
    }
    return nextWaypoint;
}

void SearchController::waypointObstacleAvoidance(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D oldGoalLocation, geometry_msgs::Pose2D alternativeLocation, std::string botName)
{
    stack_waypoints.push(oldGoalLocation);
    stack_waypoints.push(currentLocation);
    stack_waypoints.push(alternativeLocation);
}

geometry_msgs::Pose2D SearchController::waypointNextLocation(geometry_msgs::Pose2D currentLocation, std::string botName)
{
    geometry_msgs::Pose2D newGoalLocation;
    if(stack_waypoints.empty())
    {
        double newTheta = rng->uniformReal(0, 2 * M_PI); // theta between 0 and 2pi
        double newRadius = rng->uniformReal(0,1); // radius between 0 and 1
        //random new waypoint
        newGoalLocation.x = currentLocation.x + (newRadius * cos(newTheta)); //(remainingGoalDist * cos(oldGoalLocation.theta));
        newGoalLocation.y = currentLocation.y + (newRadius * sin(newTheta)); //(remainingGoalDist * sin(oldGoalLocation.theta));
    }
    else
    {
        //push new waypoint from stack
        newGoalLocation = stack_waypoints.top();
        stack_waypoints.pop();
    }
    newGoalLocation.theta = atan2(newGoalLocation.y - currentLocation.y, newGoalLocation.x - currentLocation.x);
    return newGoalLocation;
}

int SearchController::getStackSize()
{
    return stack_waypoints.size();
}
