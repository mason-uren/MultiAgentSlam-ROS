// Defines the Result type which used to standardize the information that
// can be returned by a controller object.

/* EXAMPLE:
 * 
 * This struct returns the neccesary commands to logic controller for the current controller to control the robot
 *
 * struct Result res;
 * res.type = ...;
 * res.b = ...;
 * ...
 *
 * if(res.type == behavior) {
 *      if(res.b == targetDropped) {
 *          ...
 *      } else if(res.b == ...) {
 *          ...
 *      } ...
 * } else if(res.type == waypoint) {
 *      Pose2D next = res.wpts.waypoint[0];
 *      ...
 * } else if(res.type == precisionDriving) {
 *      ...
 * }
 *
 *
 *
 */

#include <vector>

#include "Point.h"
#include <iostream>

using namespace std;

enum PIDType {
    FAST_PID, //quickest turn reasponse time
    SLOW_PID, //slower turn reasponse time
    CONST_PID //constant angular turn rate
};

enum ResultType {
    behavior, //result contains behaviour related signals for logic controller to interpret
    waypoint, //result contains waypoints for drive controller
    precisionDriving, //controller wants direct error input into drive controller
    vectorDriving, //result contains desired_heading for drive controller
    distance_driving
};

enum BehaviorTrigger {
    wait, //used by logic controller to indicate to ROSAdapter indicate when nothing should happen
    prevProcess, //when the process state should revert to the previouse state according to the controller
    noChange, //guard used by logic controller against faulty configurations
    nextProcess //when the process state should advance tot he next state according to the controller
};

struct PrecisionDriving {
    float cmdVel = 0.0; //velocity command
    float cmdAngularError = 0.0; //for the current error
    float cmdAngular = 0.0; //for const pid, angular target speed
    float setPointVel = 0.0; //set this to the target speed
    float setPointYaw = 0.0; //set this to either the target heading or 0

    float left = 0.0; //this is used by drive controller to pass PWM to ROSAdapter
    float right = 0.0;
};

struct DistanceDriving {
  float distance_desired = 0.0;
  Point start_odom;
  float completed_distance = 0.0;
};


struct Result {
    ResultType type; //stores the type of the result

    BehaviorTrigger behaviourType; //hold the behavior type
    vector<Point> waypoints;  //hold the waypoints
    PrecisionDriving pd; //holds precision commands
  DistanceDriving distance_driving_values;
  
    float fingerAngle = -1; //holds commanded for finger angle, defualt is -1 no movment
    float wristAngle = -1; //"                  " wrist angle, "                        "
    PIDType PIDMode; //hold the PID type selected for use

    bool reset; //holds a reset command where logic controller will reset the controller that asks
    float desired_heading;//contains the angle for the robot to rotate to
    bool enable_reset_center_location = false;
};
