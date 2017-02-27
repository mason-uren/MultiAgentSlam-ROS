#include "SearchController.h"

SearchController::SearchController() {
  rng = new random_numbers::RandomNumberGenerator();

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

  float waypoints_x_preliminary [] {preliminary_boundary-d, -(preliminary_boundary-d), -(preliminary_boundary-d),    preliminary_boundary-d,   preliminary_boundary-d, -(preliminary_boundary-2*d), -(preliminary_boundary-2*d),    preliminary_boundary-2*d, preliminary_boundary-2*d, -(preliminary_boundary-3*d), -(preliminary_boundary-3*d),    preliminary_boundary-3*d, preliminary_boundary-3*d, -(preliminary_boundary-4*d), -(preliminary_boundary-4*d),   preliminary_boundary-4*d, preliminary_boundary-4*d};
  float waypoints_y_preliminary [] {preliminary_boundary-d,    preliminary_boundary-d, -(preliminary_boundary-d), -(preliminary_boundary-d), preliminary_boundary-2*d,    preliminary_boundary-2*d, -(preliminary_boundary-2*d), -(preliminary_boundary-2*d), preliminary_boundary-3*d,    preliminary_boundary-3*d, -(preliminary_boundary-3*d), -(preliminary_boundary-3*d), preliminary_boundary-4*d,    preliminary_boundary-4*d, -(preliminary_boundary-4*d), -(preliminary_boundary-4*d), preliminary_boundary-5*d};

  float waypoints_x2_preliminary [] {preliminary_boundary-4*d, -(preliminary_boundary-5*d), -(preliminary_boundary-5*d),    preliminary_boundary-5*d,    preliminary_boundary-5*d, -(preliminary_boundary-6*d), -(preliminary_boundary-6*d),    preliminary_boundary-6*d, preliminary_boundary-6*d,  -(preliminary_boundary-7*d), -(preliminary_boundary-7*d),    preliminary_boundary-7*d, preliminary_boundary-7*d, -(preliminary_boundary-8*d), -(preliminary_boundary-8*d),    preliminary_boundary-8*d, preliminary_boundary-8*d, preliminary_boundary-8*d, -(preliminary_boundary-9*d), -(preliminary_boundary-9*d),   preliminary_boundary-9*d};
  float waypoints_y2_preliminary [] {preliminary_boundary-5*d,    preliminary_boundary-5*d, -(preliminary_boundary-5*d), -(preliminary_boundary-5*d),    preliminary_boundary-6*d,    preliminary_boundary-6*d, -(preliminary_boundary-6*d), -(preliminary_boundary-6*d), preliminary_boundary-7*d,     preliminary_boundary-7*d, -(preliminary_boundary-7*d), -(preliminary_boundary-7*d), preliminary_boundary-8*d,    preliminary_boundary-8*d, -(preliminary_boundary-8*d), -(preliminary_boundary-8*d), preliminary_boundary-9*d, preliminary_boundary-9*d,    preliminary_boundary-9*d, -(preliminary_boundary-9*d), -(preliminary_boundary-9*d)};

  float waypoints_x3_preliminary [] { preliminary_boundary-9*d,  -(preliminary_boundary-10*d), -(preliminary_boundary-10*d),   preliminary_boundary-10*d,  preliminary_boundary-10*d, -(preliminary_boundary-11*d), -(preliminary_boundary-11*d),    preliminary_boundary-11*d, preliminary_boundary-11*d, -(preliminary_boundary-12*d), -(preliminary_boundary-12*d),    preliminary_boundary-12*d, preliminary_boundary-12*d, -(preliminary_boundary-13*d), -(preliminary_boundary-13*d),    preliminary_boundary-13*d, preliminary_boundary-13*d, -(preliminary_boundary-14*d), -(preliminary_boundary-14*d),    preliminary_boundary-14*d, preliminary_boundary-14*d, -(preliminary_boundary-15*d), -(preliminary_boundary-15*d),    preliminary_boundary-15*d, preliminary_boundary-15*d, -(preliminary_boundary-16*d), -(preliminary_boundary-16*d),    preliminary_boundary-16*d, preliminary_boundary-16*d,  -(preliminary_boundary-17*d), -(preliminary_boundary-17*d),    preliminary_boundary-17*d, preliminary_boundary-17*d, -(preliminary_boundary-18*d), -(preliminary_boundary-18*d),    preliminary_boundary-18*d, preliminary_boundary-18*d,  -(preliminary_boundary-19*d), -(preliminary_boundary-19*d),    preliminary_boundary-19*d, preliminary_boundary-19*d};
  float waypoints_y3_preliminary [] {preliminary_boundary-10*d,     preliminary_boundary-10*d, -(preliminary_boundary-10*d), -(preliminary_boundary-10*d), preliminary_boundary-11*d,    preliminary_boundary-11*d, -(preliminary_boundary-11*d), -(preliminary_boundary-11*d), preliminary_boundary-12*d,    preliminary_boundary-12*d, -(preliminary_boundary-12*d), -(preliminary_boundary-12*d), preliminary_boundary-13*d,    preliminary_boundary-13*d, -(preliminary_boundary-13*d), -(preliminary_boundary-13*d), preliminary_boundary-14*d,    preliminary_boundary-14*d, -(preliminary_boundary-14*d), -(preliminary_boundary-14*d), preliminary_boundary-15*d,    preliminary_boundary-15*d, -(preliminary_boundary-15*d), -(preliminary_boundary-15*d), preliminary_boundary-16*d,    preliminary_boundary-16*d, -(preliminary_boundary-16*d), -(preliminary_boundary-16*d), preliminary_boundary-17*d,     preliminary_boundary-17*d, -(preliminary_boundary-17*d), -(preliminary_boundary-17*d), preliminary_boundary-18*d,    preliminary_boundary-18*d, -(preliminary_boundary-18*d), -(preliminary_boundary-18*d), preliminary_boundary-19*d,     preliminary_boundary-19*d, -(preliminary_boundary-19*d), -(preliminary_boundary-19*d), preliminary_boundary-20*d};


  // End: From old code:



  //stacks to store waypoints

  geometry_msgs::Pose2D nextPosition;
  bool finalRound = false; // We need to fix this later.  Right now I just want to get prelim working.
  //push waypoints onto stack
  if (finalRound)
  {
      for (int i = sizeof(waypoints_x_final)/sizeof(waypoints_x_final[0]); i > 0; i--)
      {
          nextPosition.x = waypoints_x_final[i];
          nextPosition.y = waypoints_y_final[i];
          stack1_final.push(nextPosition);
      }

      for (int i = sizeof(waypoints_x2_final)/sizeof(waypoints_x2_final[0]); i > 0; i--)
      {
          nextPosition.x = waypoints_x2_final[i];
          nextPosition.y = waypoints_y2_final[i];
          stack2_final.push(nextPosition);
      }

      for (int i = sizeof(waypoints_x3_final)/sizeof(waypoints_x3_final[0]); i > 0; i--)
      {
          nextPosition.x = waypoints_x3_final[i];
          nextPosition.y = waypoints_y3_final[i];
          stack3_final.push(nextPosition);
      }
  }
  else
  {
      for (int i = sizeof(waypoints_x_preliminary)/sizeof(waypoints_x_preliminary[0]); i > 0; i--)
      {
          nextPosition.x = waypoints_x_preliminary[i];
          nextPosition.y = waypoints_y_preliminary[i];
          stack1_prelim.push(nextPosition);
      }

      for (int i = sizeof(waypoints_x2_preliminary)/sizeof(waypoints_x2_preliminary[0]); i > 0; i--)
      {
          nextPosition.x = waypoints_x2_preliminary[i];
          nextPosition.y = waypoints_y2_preliminary[i];
          stack2_prelim.push(nextPosition);
      }

      for (int i = sizeof(waypoints_x3_preliminary)/sizeof(waypoints_x3_preliminary[0]); i > 0; i--)
      {
          nextPosition.x = waypoints_x3_preliminary[i];
          nextPosition.y = waypoints_y3_preliminary[i];
          stack3_prelim.push(nextPosition);
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
  //hardcoded for now
  if(botName == "ajax")
  {
    stack1_prelim.push(oldGoalLocation);
    stack1_prelim.push(currentLocation);
  }
  else if(botName == "achilles")
  {
    stack2_prelim.push(oldGoalLocation);
//    stack2_prelim.push(currentLocation);
  }
  else if(botName == "aeneas")
  {
    stack3_prelim.push(oldGoalLocation);
//    stack3_prelim.push(currentLocation);
  }
}

void SearchController::waypointObstacleAvoidance(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D oldGoalLocation, geometry_msgs::Pose2D alternativeLocation, std::string botName)
{
  //hardcoded for now
  if(botName == "ajax")
  {
    stack1_prelim.push(oldGoalLocation);
    //stack1_prelim.push(currentLocation);
    stack1_prelim.push(alternativeLocation);
  }
  else if(botName == "achilles")
  {
    stack2_prelim.push(oldGoalLocation);
    //stack2_prelim.push(currentLocation);
    stack2_prelim.push(alternativeLocation);
  }
  else if(botName == "aeneas")
  {
    stack3_prelim.push(oldGoalLocation);
    //stack3_prelim.push(currentLocation);
    stack3_prelim.push(alternativeLocation);
  }
}

geometry_msgs::Pose2D SearchController::waypointNextLocation(geometry_msgs::Pose2D currentLocation, std::string botName)
{
  geometry_msgs::Pose2D newGoalLocation;
  //hardcoded for now
  if(botName == "ajax")
  {
      newGoalLocation = GetNewGoalLocation(stack1_prelim, currentLocation);
  }
  else if(botName == "achilles")
  {
      newGoalLocation = GetNewGoalLocation(stack2_prelim, currentLocation);
  }
  else if(botName == "aeneas")
  {
      newGoalLocation = GetNewGoalLocation(stack3_prelim, currentLocation);
  }

  return newGoalLocation;
}

geometry_msgs::Pose2D SearchController::GetNewGoalLocation(std::stack<geometry_msgs::Pose2D> &wp_stack, geometry_msgs::Pose2D currentLocation)
{
    geometry_msgs::Pose2D newGoalLocation;
    if(wp_stack.empty())
    {
        //random new waypoint
        newGoalLocation.x = currentLocation.x + (0.50 * cos(currentLocation.theta)); //(remainingGoalDist * cos(oldGoalLocation.theta));
        newGoalLocation.y = currentLocation.y + (0.50 * sin(currentLocation.theta)); //(remainingGoalDist * sin(oldGoalLocation.theta));
    }
    else
    {
        //push new waypoint from stack
        newGoalLocation = wp_stack.top();
        wp_stack.pop();
    }
    newGoalLocation.theta = atan2(newGoalLocation.y - currentLocation.y, newGoalLocation.x - currentLocation.x);
    return newGoalLocation;
}
