#include <ros/ros.h>

// ROS libraries
#include <angles/angles.h>
#include <random_numbers/random_numbers.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// ROS messages
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <std_msgs/Float32MultiArray.h>
#include "swarmie_msgs/Waypoint.h"
#include "swarmie_msgs/Recruitment.h"
#include "swarmie_msgs/Skid.h"

// Include Controllers
#include "LogicController.h"
#include "Controller.h"
#include <vector>
#include <cmath>
#include <math.h>
#include <algorithm> 

#include "Point.h"
#include "Tag.h"

#include "PositionPublisher.hpp"

// To handle shutdown signals so the node quits
// properly in response to "rosnode kill"
#include <ros/ros.h>
#include <signal.h>

#include <exception> // For exception handling


#include <angles/angles.h>
#include "std_msgs/Float64MultiArray.h"

#include <fstream>
#include <iostream>

using namespace std;

// Define Exceptions
// Define an exception to be thrown if the user tries to create
// a RangeShape using invalid dimensions
class ROSAdapterRangeShapeInvalidTypeException : public std::exception {
public:
  ROSAdapterRangeShapeInvalidTypeException(std::string msg) {
    this->msg = msg;
  }
  
  virtual const char* what() const throw()
  {
    std::string message = "Invalid RangeShape type provided: " + msg;
    return message.c_str();
  }
  
private:
  std::string msg;
};


// Random number generator
random_numbers::RandomNumberGenerator* rng;

std_msgs::UInt8 collision_msg;
// Create logic controller

LogicController logicController;

void humanTime();	//translates time into human time

// Behaviours Logic Functions
void sendDriveCommand(double linear_velocity, double angular_velocity);
void openFingers(); 	// Open fingers to 90 degrees
void closeFingers();	// Close fingers to 0 degrees
void raiseWrist();  	// Return wrist back to 0 degrees
void lowerWrist();  	// Lower wrist to 50 degrees
void centerWrist();
void resultHandler();	// Not Used/Dead Code, prototype has no definition


Point updateCenterLocation();		//calls transformMapCenterToOdom, returns a center location in ODOM frame
void transformMapCentertoOdom();	//checks ODOMs perceived idea of where the center is with a stored GPS center coordinate and adjusts ODOM center value to account for drift

PositionPublisher* positionPublisher;

// Numeric Variables for rover positioning
geometry_msgs::Pose2D current_location_odom;		//current location using ODOM
geometry_msgs::Pose2D current_location_ekf;	//current location using GPS
geometry_msgs::Pose2D current_location_avg;	//an average of the robots current location
Point location_offset_odom;
Point location_offset_ekf;


geometry_msgs::Pose2D centerLocation;		//Not used, dead code
geometry_msgs::Pose2D centerLocationMap;	//A GPS point of the center location, used to help reduce drift from ODOM
geometry_msgs::Pose2D centerLocationOdom;	//The centers location based on ODOM
geometry_msgs::Pose2D centerLocationMapRef;	//Variable used in TransformMapCenterToOdom, can be moved to make it local instead of global

int current_mode = 0;

const float BEHAVIOR_LOOP_TIME_STEP = 0.1; 	//time between the behaviour loop calls
const float STATUS_PUBLISH_INTERVAL = 1;	//time between publishes
const float HEARTBEAT_PUBLISH_INTERVAL = 2;	//time between heartbeat publishes
const float waypointTolerance = 0.1; 		//10 cm tolerance.
const float CAMERA_OFFSET_CORRECTION = 0.020;
const float CAMERA_HEIGHT_IN_CM = 0.195;

// used for calling code once but not in main
bool initilized = false;	//switched to true after running through state machine the first time, initializes base values

//bool dropOffMode = false;	//switched to true after starting dropOff state

float linear_velocity = 0;	//forward speed, POSITIVE = forward, NEGATIVE = backward
float angular_velocity = 0;	//turning speed, POSITIVE = left, NEGATIVE = right

float prevWrist = 0;	//last wrist angle
float prevFinger = 0;	//last finger angle
long int start_time = 0;	//stores time when robot is swtiched on
float minutes_time = 0;	//time in minutes
float hours_time = 0;	//time in hours

float drift_tolerance = 0.5; // the perceived difference between ODOM and GPS values before shifting the values up or down, in meters

Result result;		//result struct for passing and storing values to drive robot

std_msgs::String msg;	//used for passing messages to the GUI

geometry_msgs::Twist velocity;
char host[128];		//rovers hostname
string published_name;	//published hostname



char prev_state_machine[128];

// Publishers
ros::Publisher state_machine_publisher;		//publishes state machine status
ros::Publisher status_publisher;		//publishes rover status
ros::Publisher finger_angle_publisher;		//publishes gripper angle to move gripper fingers
ros::Publisher wrist_angle_publisher;		//publishes wrist angle to move wrist
ros::Publisher infoLogPublisher;		//publishes a message to the infolog box on GUI
ros::Publisher drive_control_publisher;		//publishes motor commands to the motors
ros::Publisher heartbeat_publisher;		//publishes ROSAdapters status via its "heartbeat"
ros::Publisher obstaclePublisher;
// Publishes swarmie_msgs::Waypoint messages on "/<robot>/waypooints"
// to indicate when waypoints have been reached.
ros::Publisher waypointFeedbackPublisher;	//publishes a waypoint to travel to if the rover is given a waypoint in manual mode
ros::Publisher claim_anchor_point_publisher;
ros::Publisher unclaim_anchor_point_publisher;
ros::Publisher recruitment_publisher;

// Subscribers
ros::Subscriber joystick_subscriber;			//receives joystick information
ros::Subscriber mode_subscriber; 		//receives mode from GUI
ros::Subscriber april_tag_subscriber;		//receives tag data
ros::Subscriber odometry_subscriber;		//receives ODOM data
ros::Subscriber ekf_subscriber;			//receives GPS data
ros::Subscriber virtualFenceSubscriber;		//receives data for vitrual boundaries
// manualWaypointSubscriber listens on "/<robot>/waypoints/cmd" for
// swarmie_msgs::Waypoint messages.
ros::Subscriber manualWaypointSubscriber; 	//receives manual waypoints given from GUI
ros::Subscriber recruitmentSubscriber;
ros::Subscriber claimed_anchor_point_subscriber;
ros::Subscriber unclaimed_anchor_point_subscriber;
ros::Subscriber recruitment_subscriber;

// Timers
ros::Timer state_machine_timer;
ros::Timer publish_status_timer;
ros::Timer publish_heartbeat_timer;

// records time for delays in sequenced actions, 1 second resolution.
time_t timer_start_time;

// An initial delay to allow the rover to gather enough position data to 
// average its location.
unsigned int start_delay_in_seconds = 30;
float timer_time_elapsed = 0;

//Transforms
tf::TransformListener *transform_listener;

// OS Signal Handler
void SigintEventHandler(int signal);

//Callback handlers
void JoystickCommandHandler(const sensor_msgs::Joy::ConstPtr& message);				//for joystick control
void ModeHandler(const std_msgs::UInt8::ConstPtr& message);				//for detecting which mode the robot needs to be in
// TODO: Refactor names below this line to match surf code.
void targetHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& tagInfo);	//receives and stores April Tag Data using the TAG class
void odometryHandler(const nav_msgs::Odometry::ConstPtr& message);			//receives and stores ODOM information
void mapHandler(const nav_msgs::Odometry::ConstPtr& message);				//receives and stores GPS information
void virtualFenceHandler(const std_msgs::Float32MultiArray& message);			//Used to set an invisible boundary for robots to keep them from traveling outside specific bounds
void manualWaypointHandler(const swarmie_msgs::Waypoint& message);			//Receives a waypoint (from GUI) and sets the coordinates
void behaviourStateMachine(const ros::TimerEvent&);					//Upper most state machine, calls logic controller to perform all actions
void publishStatusTimerEventHandler(const ros::TimerEvent& event);			//Publishes "ONLINE" when rover is successfully connected
void publishHeartBeatTimerEventHandler(const ros::TimerEvent& event);			
void sonarHandler(const sensor_msgs::Range::ConstPtr& sonarLeft, const sensor_msgs::Range::ConstPtr& sonarCenter, const sensor_msgs::Range::ConstPtr& sonarRight);	//handles ultrasound data and stores data
void recruitmentHandler(const swarmie_msgs::Recruitment& msg);

// Converts the time passed as reported by ROS (which takes Gazebo simulation rate into account) into milliseconds as an integer.
long int getROSTimeInMilliSecs();

int main(int argc, char **argv) {
  
  gethostname(host, sizeof (host));
  string hostname(host);
  
  if (argc >= 2) {
    published_name = argv[1];
    cout << "Welcome to the world of tomorrow " << published_name
         << "!  Behaviour turnDirectionule started." << endl;
  } else {
    published_name = hostname;
    cout << "No Name Selected. Default is: " << published_name << endl;
  }
  
  // NoSignalHandler so we can catch SIGINT ourselves and shutdown the node
  ros::init(argc, argv, (published_name + "_BEHAVIOUR"), ros::init_options::NoSigintHandler);
  ros::NodeHandle mNH;
  positionPublisher = new PositionPublisher(mNH, published_name);
  
  // Register the SIGINT event handler so the node can shutdown properly
  signal(SIGINT, SigintEventHandler);
  
  //subscribers
  joystick_subscriber = mNH.subscribe((published_name + "/joystick"), 10, JoystickCommandHandler);					//receives joystick information
  mode_subscriber = mNH.subscribe((published_name + "/mode"), 1, ModeHandler);						//receives mode from GUI
  april_tag_subscriber = mNH.subscribe((published_name + "/targets"), 10, targetHandler);					//receives tag data
  odometry_subscriber = mNH.subscribe((published_name + "/odom/filtered"), 10, odometryHandler);				//receives ODOM data
  ekf_subscriber = mNH.subscribe((published_name + "/odom/ekf"), 10, mapHandler);						//receives GPS data
  virtualFenceSubscriber = mNH.subscribe(("/virtualFence"), 10, virtualFenceHandler);					//receives data for vitrual boundaries
  manualWaypointSubscriber = mNH.subscribe((published_name + "/waypoints/cmd"), 10, manualWaypointHandler);		//receives manual waypoints given from GUI
  message_filters::Subscriber<sensor_msgs::Range> sonarLeftSubscriber(mNH, (published_name + "/sonarLeft"), 10);
  message_filters::Subscriber<sensor_msgs::Range> sonarCenterSubscriber(mNH, (published_name + "/sonarCenter"), 10);
  message_filters::Subscriber<sensor_msgs::Range> sonarRightSubscriber(mNH, (published_name + "/sonarRight"), 10);
  recruitmentSubscriber = mNH.subscribe("/detectionLocations", 10, recruitmentHandler);

  //publishers
  status_publisher = mNH.advertise<std_msgs::String>((published_name + "/swarmie_status"), 1, true);			//publishes rover status
  obstaclePublisher = mNH.advertise<std_msgs::UInt8>((published_name + "/obstacle"), 10, true);
  state_machine_publisher = mNH.advertise<std_msgs::String>((published_name + "/state_machine"), 1, true);			//publishes state machine status
  finger_angle_publisher = mNH.advertise<std_msgs::Float32>((published_name + "/fingerAngle/cmd"), 1, true);			//publishes gripper angle to move gripper finger
  wrist_angle_publisher = mNH.advertise<std_msgs::Float32>((published_name + "/wristAngle/cmd"), 1, true);			//publishes wrist angle to move wrist
  infoLogPublisher = mNH.advertise<std_msgs::String>("/infoLog", 1, true);						//publishes a message to the infolog box on GUI
  drive_control_publisher = mNH.advertise<swarmie_msgs::Skid>((published_name + "/driveControl"), 10);			//publishes motor commands to the motors
  heartbeat_publisher = mNH.advertise<std_msgs::String>((published_name + "/behaviour/heartbeat"), 1, true);		//publishes ROSAdapters status via its "heartbeat"
  waypointFeedbackPublisher = mNH.advertise<swarmie_msgs::Waypoint>((published_name + "/waypoints"), 1, true);		//publishes a waypoint to travel to if the rover is given a waypoint in manual mode

  //timers
  publish_status_timer = mNH.createTimer(ros::Duration(STATUS_PUBLISH_INTERVAL), publishStatusTimerEventHandler);
  state_machine_timer = mNH.createTimer(ros::Duration(BEHAVIOR_LOOP_TIME_STEP), behaviourStateMachine);
  
  publish_heartbeat_timer = mNH.createTimer(ros::Duration(HEARTBEAT_PUBLISH_INTERVAL), publishHeartBeatTimerEventHandler);
  
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Range, sensor_msgs::Range, sensor_msgs::Range> sonarSyncPolicy;
  
  message_filters::Synchronizer<sonarSyncPolicy> sonarSync(sonarSyncPolicy(10), sonarLeftSubscriber, sonarCenterSubscriber, sonarRightSubscriber);
  sonarSync.registerCallback(boost::bind(&sonarHandler, _1, _2, _3));
  
  transform_listener = new tf::TransformListener();
  std_msgs::String msg;
  msg.data = "Log Started";
  infoLogPublisher.publish(msg);
  
  stringstream ss;
  ss << "Rover start delay set to " << start_delay_in_seconds << " seconds";
  msg.data = ss.str();
  infoLogPublisher.publish(msg);

  if(current_mode != 2 && current_mode != 3)
  {
    // ensure the logic controller starts in the correct mode.
    logicController.SetModeManual();
  }

  timer_start_time = time(0);
  
  ros::spin();

  delete positionPublisher;
  return EXIT_SUCCESS;
}


// This is the top-most logic control block organised as a state machine.
// This function calls the dropOff, pickUp, and search controllers.
// This block passes the goal location to the proportional-integral-derivative
// controllers in the abridge package.
void behaviourStateMachine(const ros::TimerEvent&)
{

  std_msgs::String stateMachineMsg;
  
  // time since timer_start_time was set to current time
  timer_time_elapsed = time(0) - timer_start_time;
  
  // init code goes here. (code that runs only once at start of
  // auto mode but wont work in main goes here)
  if (!initilized)
  {

    if (timer_time_elapsed > start_delay_in_seconds)
    {

      // initialization has run
      initilized = true;
      //TODO: this just sets center to 0 over and over and needs to change
      Point centerOdom;
      centerOdom.x = 1.3 * cos(current_location_odom.theta);
      centerOdom.y = 1.3 * sin(current_location_odom.theta);
      centerOdom.theta = centerLocation.theta;
      logicController.SetCenterLocationOdom(centerOdom);
      
      Point centerMap;
      centerMap.x = current_location_ekf.x + (1.3 * cos(current_location_ekf.theta));
      centerMap.y = current_location_ekf.y + (1.3 * sin(current_location_ekf.theta));
      centerMap.theta = centerLocationMap.theta;
      logicController.SetCenterLocationMap(centerMap);
      
      centerLocationMap.x = centerMap.x;
      centerLocationMap.y = centerMap.y;
      
      centerLocationOdom.x = centerOdom.x;
      centerLocationOdom.y = centerOdom.y;
      
      start_time = getROSTimeInMilliSecs();
    }

    else
    {
      return;
    }
    
  }

  // Robot is in autonomous mode
  if (current_mode == 2 || current_mode == 3)
  {
    
    humanTime();
    
    //update the time used by all the controllers, logic controller will send to other controllers
    logicController.SetCurrentTimeInMilliSecs( getROSTimeInMilliSecs() );
    
    //update center location, logic controller will send to other controllers
    logicController.SetCenterLocationOdom( updateCenterLocation() );
    
    //ask logic controller for the next set of actuator commands
    result = logicController.DoWork();
    
    bool wait = false;	//a variable created to check if we are in a waiting state
    
    //if a wait behaviour is thrown sit and do nothing untill logicController is ready
    if (result.type == behavior)
    {
      if (result.b == wait)
      {
        wait = true;
      }
    }
    
    //do this when wait behaviour happens
    if (wait)
    {
      sendDriveCommand(0.0,0.0);
      std_msgs::Float32 angle;
      
      angle.data = prevFinger;
      finger_angle_publisher.publish(angle);
      angle.data = prevWrist;
      wrist_angle_publisher.publish(angle);
    }
    
    //normally interpret logic controllers actuator commands and deceminate them over the appropriate ROS topics
    else
    {
      
      sendDriveCommand(result.pd.left,result.pd.right);	//uses the results struct with data sent back from logic controller to send motor commands
      

      //Alter finger and wrist angle is told to reset with last stored value if currently has -1 value
      std_msgs::Float32 angle;
      if (result.fingerAngle != -1)
      {
        angle.data = result.fingerAngle;	//uses results struct with data sent back from logic controller to get angle data
        finger_angle_publisher.publish(angle);	//publish angle data to the gripper fingers
        prevFinger = result.fingerAngle;	//store the last known gripper finger angle
      }

      if (result.wristAngle != -1)
      {
        angle.data = result.wristAngle;		//uses results struct with data sent back from logic controller to get angle data
        wrist_angle_publisher.publish(angle);	//publish angle data to the gripper wrist
        prevWrist = result.wristAngle;		//store the last known gripper wrist angle
      }
    }
  collision_msg.data = logicController.getCollisionCalls();
  obstaclePublisher.publish(collision_msg);
    //publishHandeling here
    //logicController.getPublishData(); //Not Currently Implemented, used to get data from logic controller and publish to the appropriate ROS Topic; Suggested
    
    
    //adds a blank space between sets of debugging data to easily tell one tick from the next
    cout << endl;
    
  }
  
  // mode is NOT auto
  else	//manual mode
  {
    humanTime();

    logicController.SetCurrentTimeInMilliSecs( getROSTimeInMilliSecs() );

    // publish current state for the operator to see
    stateMachineMsg.data = "WAITING";

    // ask the logicController to get the waypoints that have been
    // reached.
    std::vector<int> cleared_waypoints = logicController.GetClearedWaypoints();

    for(std::vector<int>::iterator it = cleared_waypoints.begin();
        it != cleared_waypoints.end(); it++)
    {
      swarmie_msgs::Waypoint wpt;
      wpt.action = swarmie_msgs::Waypoint::ACTION_REACHED;
      wpt.id = *it;
      waypointFeedbackPublisher.publish(wpt);
    }
    result = logicController.DoWork();	//ask logic controller to run
    if(result.type != behavior || result.b != wait)
    {
      // if the logic controller requested that the robot drive, then
      // drive. Otherwise there are no manual waypoints and the robot
      // should sit idle. (ie. only drive according to joystick
      // input).
      sendDriveCommand(result.pd.left,result.pd.right);
    }
  }

  // publish state machine string for user, only if it has changed, though
  if (strcmp(stateMachineMsg.data.c_str(), prev_state_machine) != 0)
  {
    state_machine_publisher.publish(stateMachineMsg);
    sprintf(prev_state_machine, "%s", stateMachineMsg.data.c_str());
  }
}

void sendDriveCommand(double left, double right)
{
  swarmie_msgs::Skid skid_command;
  skid_command.left  = left;
  skid_command.right = right;
  // publish the drive commands
  drive_control_publisher.publish(skid_command);
}

/*************************
 * ROS CALLBACK HANDLERS *
 *************************/

void targetHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message) {

  // Don't pass April tag data to the logic controller if the robot is not in autonomous mode.
  // This is to make sure autonomous behaviours are not triggered while the rover is in manual mode. 
  if(current_mode == 0 || current_mode == 1) 
  { 
    return; 
  }

  if (message->detections.size() > 0) {
    vector<Tag> tags;

    for (int i = 0; i < message->detections.size(); i++) {

      // Package up the ROS AprilTag data into our own type that does not rely on ROS.
      Tag loc;
      loc.setID( message->detections[i].id );

      // Pass the position of the AprilTag
      geometry_msgs::PoseStamped tagPose = message->detections[i].pose;
      loc.setPosition( make_tuple( tagPose.pose.position.x,
				   tagPose.pose.position.y,
				   tagPose.pose.position.z ) );

      // Pass the orientation of the AprilTag
      loc.setOrientation( ::boost::math::quaternion<float>( tagPose.pose.orientation.x,
							    tagPose.pose.orientation.y,
							    tagPose.pose.orientation.z,
							    tagPose.pose.orientation.w ) );
      tags.push_back(loc);
    }

    // To enable recruitment uncomment these lines.
    // Point curr_loc;
    // curr_loc.x = current_location_ekf.x;
    // curr_loc.y = current_location_ekf.y;
    // positionPublisher->setDetections(tags, curr_loc);

    logicController.SetAprilTags(tags);
  }
  
}

void ModeHandler(const std_msgs::UInt8::ConstPtr& message) {
  current_mode = message->data;
  if(current_mode == 2 || current_mode == 3) {
    logicController.SetModeAuto();
  }
  else {
    logicController.SetModeManual();
  }
  sendDriveCommand(0.0, 0.0);
}

void sonarHandler(const sensor_msgs::Range::ConstPtr& sonarLeft, const sensor_msgs::Range::ConstPtr& sonarCenter, const sensor_msgs::Range::ConstPtr& sonarRight) {
  
  logicController.SetSonarData(sonarLeft->range, sonarCenter->range, sonarRight->range);
  
}

void odometryHandler(const nav_msgs::Odometry::ConstPtr& message) {
  //Get (x,y) location directly from pose
  current_location_odom.x = message->pose.pose.position.x;
  current_location_odom.y = message->pose.pose.position.y;
  
  //Get theta rotation by converting quaternion orientation to pitch/roll/yaw
  tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  current_location_odom.theta = yaw;
  
  linear_velocity = message->twist.twist.linear.x;
  angular_velocity = message->twist.twist.angular.z;
  
  
  Point currentLoc;
  currentLoc.x = current_location_odom.x;
  currentLoc.y = current_location_odom.y;
  currentLoc.theta = current_location_odom.theta;
  logicController.SetPositionData(currentLoc);
  logicController.SetVelocityData(linear_velocity, angular_velocity);
}

// Allows a virtual fence to be defined and enabled or disabled through ROS
void virtualFenceHandler(const std_msgs::Float32MultiArray& message) 
{
  // Read data from the message array
  // The first element is an integer indicating the shape type
  // 0 = Disable the virtual fence
  // 1 = circle
  // 2 = rectangle
  int shape_type = static_cast<int>(message.data[0]); // Shape type
  
  if (shape_type == 0)
  {
    logicController.setVirtualFenceOff();
  }
  else
  {
    // Elements 2 and 3 are the x and y coordinates of the range center
    Point center;
    center.x = message.data[1]; // Range center x
    center.y = message.data[2]; // Range center y
    
    // If the shape type is "circle" then element 4 is the radius, if rectangle then width
    switch ( shape_type )
    {
    case 1: // Circle
    {
      if ( message.data.size() != 4 ) throw ROSAdapterRangeShapeInvalidTypeException("Wrong number of parameters for circle shape type in ROSAdapter.cpp:virtualFenceHandler()");
      float radius = message.data[3]; 
      logicController.setVirtualFenceOn( new RangeCircle(center, radius) );
      break;
    }
    case 2: // Rectangle 
    {
      if ( message.data.size() != 5 ) throw ROSAdapterRangeShapeInvalidTypeException("Wrong number of parameters for rectangle shape type in ROSAdapter.cpp:virtualFenceHandler()");
      float width = message.data[3]; 
      float height = message.data[4]; 
      logicController.setVirtualFenceOn( new RangeRectangle(center, width, height) );
      break;
    }
    default:
    { // Unknown shape type specified
      throw ROSAdapterRangeShapeInvalidTypeException("Unknown Shape type in ROSAdapter.cpp:virtualFenceHandler()");
    }
    }
  }
}

void mapHandler(const nav_msgs::Odometry::ConstPtr& message) {
  //Get (x,y) location directly from pose
  current_location_ekf.x = message->pose.pose.position.x;
  current_location_ekf.y = message->pose.pose.position.y;
  
  //Get theta rotation by converting quaternion orientation to pitch/roll/yaw
  tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  current_location_ekf.theta = yaw;
  
  linear_velocity = message->twist.twist.linear.x;
  angular_velocity = message->twist.twist.angular.z;
  
  Point curr_loc;
  curr_loc.x = current_location_ekf.x;
  curr_loc.y = current_location_ekf.y;
  curr_loc.theta = current_location_ekf.theta;
  logicController.SetMapPositionData(curr_loc);
  logicController.SetMapVelocityData(linear_velocity, angular_velocity);
}

void JoystickCommandHandler(const sensor_msgs::Joy::ConstPtr& message) {
  const int max_motor_cmd = 255;
  if (current_mode == 0 || current_mode == 1) {	//takes data coming from joystick and stores into linear and angular variables
    float linear  = abs(message->axes[4]) >= 0.1 ? message->axes[4]*max_motor_cmd : 0.0;
    float angular = abs(message->axes[3]) >= 0.1 ? message->axes[3]*max_motor_cmd : 0.0;

    float left = linear - angular;
    float right = linear + angular;
    //check to see if commands exceed MAX values, and if so set them to hard coded MAX value
    if(left > max_motor_cmd) {	
      left = max_motor_cmd;
    }
    else if(left < -max_motor_cmd) {
      left = -max_motor_cmd;
    }

    if(right > max_motor_cmd) {
      right = max_motor_cmd;
    }
    else if(right < -max_motor_cmd) {
      right = -max_motor_cmd;
    }

    sendDriveCommand(left, right);
  }
}


void publishStatusTimerEventHandler(const ros::TimerEvent&) {
  std_msgs::String msg;
  msg.data = "online";		//change this with team name
  status_publisher.publish(msg);
}

void manualWaypointHandler(const swarmie_msgs::Waypoint& message) {
  Point wp;
  wp.x = message.x;
  wp.y = message.y;
  wp.theta = 0.0;
  switch(message.action) {
  case swarmie_msgs::Waypoint::ACTION_ADD:
    logicController.AddManualWaypoint(wp, message.id);
    break;
  case swarmie_msgs::Waypoint::ACTION_REMOVE:
    logicController.RemoveManualWaypoint(message.id);
    break;
  }
}

void recruitmentHandler(const swarmie_msgs::Recruitment& msg)
{
   if(msg.name.data != published_name) {
      Point p;
      p.x = msg.x;
      p.y = msg.y;
      logicController.gotRecruitmentMessage(p);
   }
}

void SigintEventHandler(int sig) {
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

void publishHeartBeatTimerEventHandler(const ros::TimerEvent&) {
  std_msgs::String msg;
  msg.data = "";
  heartbeat_publisher.publish(msg);
}

long int getROSTimeInMilliSecs()
{
  // Get the current time according to ROS (will be zero for simulated clock until the first time message is recieved).
  ros::Time t = ros::Time::now();
  
  // Convert from seconds and nanoseconds to milliseconds.
  return t.sec*1e3 + t.nsec/1e6;
  
}


Point updateCenterLocation()
{
  transformMapCentertoOdom();
  
  Point tmp;
  tmp.x = centerLocationOdom.x;
  tmp.y = centerLocationOdom.y;
  
  return tmp;
}

void transformMapCentertoOdom()
{
  
  // map frame
  geometry_msgs::PoseStamped mapPose;
  
  // setup msg to represent the center location in map frame
  mapPose.header.stamp = ros::Time::now();
  
  mapPose.header.frame_id = published_name + "/map";
  mapPose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, centerLocationMap.theta);
  mapPose.pose.position.x = centerLocationMap.x;
  mapPose.pose.position.y = centerLocationMap.y;
  geometry_msgs::PoseStamped odomPose;
  string x = "";
  
  try
  { //attempt to get the transform of the center point in map frame to odom frame.
    transform_listener->waitForTransform(published_name + "/map", published_name + "/odom", ros::Time::now(), ros::Duration(1.0));
    transform_listener->transformPose(published_name + "/odom", mapPose, odomPose);
  }
  
  catch(tf::TransformException& ex) {  //bad transform
    ROS_INFO("Received an exception trying to transform a point from \"map\" to \"odom\": %s", ex.what());
    x = "Exception thrown " + (string)ex.what();
    std_msgs::String msg;
    stringstream ss;
    ss << "Exception in mapAverage() " + (string)ex.what();
    msg.data = ss.str();
    infoLogPublisher.publish(msg);
    cout << msg.data << endl;
  }
  
  // Use the position and orientation provided by the ros transform.
  centerLocationMapRef.x = odomPose.pose.position.x; //set centerLocation in odom frame
  centerLocationMapRef.y = odomPose.pose.position.y;
  
 // cout << "x ref : "<< centerLocationMapRef.x << " y ref : " << centerLocationMapRef.y << endl;
  
  float xdiff = centerLocationMapRef.x - centerLocationOdom.x;	//get difference in X values
  float ydiff = centerLocationMapRef.y - centerLocationOdom.y;	//get difference in Y values
  
  float diff = hypot(xdiff, ydiff);	//get total difference
  
  if (diff > drift_tolerance)	//If the difference is greater than tolerance, adjust the rovers perceived idea of where the center is. Used to decrease ODOM drift and keep rover accuracy for longer periods of time
  {
    centerLocationOdom.x += xdiff/diff;	//adjust X
    centerLocationOdom.y += ydiff/diff;	//adjust Y
  }
  
  //cout << "center x diff : " << centerLocationMapRef.x - centerLocationOdom.x << " center y diff : " << centerLocationMapRef.y - centerLocationOdom.y << endl;
  //cout << hypot(centerLocationMapRef.x - centerLocationOdom.x, centerLocationMapRef.y - centerLocationOdom.y) << endl;
          
}

void humanTime() {
  
  float timeDiff = (getROSTimeInMilliSecs()-start_time)/1e3;
  if (timeDiff >= 60) {
    minutes_time++;
    start_time += 60  * 1e3;
    if (minutes_time >= 60) {
      hours_time++;
      minutes_time -= 60;
    }
  }
  timeDiff = floor(timeDiff*10)/10;
  
  double intP, frac;
  frac = modf(timeDiff, &intP);
  timeDiff -= frac;
  frac = round(frac*10);
  if (frac > 9) {
    frac = 0;
  }
  
  //cout << "System has been Running for :: " << hours_time << " : hours " << minutes_time << " : minutes " << timeDiff << "." << frac << " : seconds" << endl; //you can remove or comment this out it just gives indication something is happening to the log file
}
