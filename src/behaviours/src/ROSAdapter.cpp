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
//#include "LogicController.h"
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

//LogicController logicController;

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
//ros::Publisher claim_anchor_point_publisher;
//ros::Publisher unclaim_anchor_point_publisher;
ros::Publisher recruitment_publisher;

// Subscribers
ros::Subscriber joystick_subscriber;			//receives joystick information
ros::Subscriber mode_subscriber; 		//receives mode from GUI
ros::Subscriber april_tag_subscriber;		//receives tag data
ros::Subscriber odometry_subscriber;		//receives ODOM data
ros::Subscriber ekf_subscriber;			//receives GPS data
//ros::Subscriber virtualFenceSubscriber;		//receives data for vitrual boundaries
// manualWaypointSubscriber listens on "/<robot>/waypoints/cmd" for
// swarmie_msgs::Waypoint messages.
//ros::Subscriber manualWaypointSubscriber; 	//receives manual waypoints given from GUI
ros::Subscriber recruitment_subscriber;
//ros::Subscriber claimed_anchor_point_subscriber;
//ros::Subscriber unclaimed_anchor_point_subscriber;

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
void AprilTagHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& tagInfo);	//receives and stores April Tag Data using the TAG class
void OdometryHandler(const nav_msgs::Odometry::ConstPtr& message);			//receives and stores ODOM information
void EKFHandler(const nav_msgs::Odometry::ConstPtr& message);				//receives and stores GPS information
// TODO: Figure out what virtualFenceHandler and manualWaypointHandler are and how to migrate them
//void virtualFenceHandler(const std_msgs::Float32MultiArray& message);			//Used to set an invisible boundary for robots to keep them from traveling outside specific bounds
//void manualWaypointHandler(const swarmie_msgs::Waypoint& message);			//Receives a waypoint (from GUI) and sets the coordinates
void PublishStatusTimerEventHandler(const ros::TimerEvent& event);			//Publishes "ONLINE" when rover is successfully connected
void PublishHeartBeatTimerEventHandler(const ros::TimerEvent& event);			
void SonarHandler(const sensor_msgs::Range::ConstPtr& sonarLeft, const sensor_msgs::Range::ConstPtr& sonarCenter, const sensor_msgs::Range::ConstPtr& sonarRight);	//handles ultrasound data and stores data
void RecruitmentHandler(const swarmie_msgs::Recruitment& msg);

void BehaviourStateMachine(const ros::TimerEvent&);					//Upper most state machine, calls logic controller to perform all actions

// Converts the time passed as reported by ROS (which takes Gazebo simulation rate into account) into milliseconds as an integer.
long int GetROSTimeInMilliSecs();
void ActuatorOutput(float linear_velocity_out, float angular_velocity_out);
int SaturationCheck(int direction, int sat);
float Wrap(float theta);
float GetNewHeading(int side);
Point ClosestAnchorPoint();
bool CheckDriveDistance(float desired_distance);
void SubtractLocationOffset();
void AddLocationOffset();
bool PickupBreakout();
bool AnchorPointBreakout();
bool DropOffBreakout();
bool ClusterBreakout();
void ResetFlags();
int ContainsClaimedAnchorPoints(geometry_msgs::Pose2D anchor_point);
void StateLogger(string state);

#define INIT 0
#define SEARCH 1
#define SEARCH_OBSTACLE_AVOIDANCE  2
#define PICKUP_ALIGN 3
#define PICKUP_ROTATE_90 4
#define PICKUP_FORWARD 5
#define PICKUP_BLIND_DRIVE 6
#define PICKUP_CLOSE_GRIPPER 7
#define PICKUP_BACKUP_RAISE_GRIPPER 8
#define PICKUP_WAIT_TO_RAISE_GRIPPER 9
#define ANCHOR_POINT_ALIGN 10
#define ANCHOR_POINT_FORWARD 11
#define RESOURCE_HELD_CHECK 12
#define GO_HOME_ALIGN 13
#define GO_HOME_DRIVE 14
#define RESOURCES_COVERING_HOME_ALIGN 15
#define RESOURCES_COVERING_HOME_FORWARD 16
#define RESOURCES_COVERING_HOME_BLIND_DRIVE 17
#define RESOURCES_COVERING_HOME_BACKUP 18
#define SEARCH_HOME_ROTATE 19
#define SEARCH_HOME_FORWARD 20
#define SEARCH_HOME_OBSTACLE_WAIT 21
#define SEARCH_HOME 22
#define SEARCH_HOME_OBSTACLE_AVOIDANCE 23
#define ANCHOR_POINT_TRAVERSE_ROTATE 24
#define ANCHOR_POINT_TRAVERSE_FORWARD 25
#define GO_HOME_TRAVERSE_ROTATE 26
#define GO_HOME_TRAVERSE_FORWARD 27
#define DROPOFF_ALIGN 28
#define DROPOFF_FORWARD 29
#define DROPOFF_BACKUP_OPEN_GRIPPER 30
#define CLUSTER_ALIGN 31
#define CLUSTER_FORWARD 32
#define CLUSTER_TRAVERSE_ROTATE 33
#define CLUSTER_TRAVERSE_FORWARD 34
//#define CLUSTER_SWEEP 35

const bool SIMULATOR = true;

//Research Flags
const bool ANCHOR_POINTS = false;
const bool SITE_FIDELITY = true;
const bool RECRUITMENT = false;


bool lost_home = false;
bool inside_home = false;


int state_machine_state = INIT;
bool obstacle_encountered = false;
bool home_encountered = false;
bool resource_encountered = false;
bool resource_held = false;


//higher K values means faster movement
float SEARCH_K = 1;
const float OBS_K = .2;
const float PICKUP_ALIGN_K = .1;
const float PICKUP_FORWARD_K = .2;
const float PICKUP_BLIND_K = .2;
const float PICKUP_BACKUP_K = .3;
const float ROTATE_90_K = .1;
const float ANCHOR_POINT_ALIGN_K = .2;
const float ANCHOR_POINT_FORWARD_K = .25;
const float ANCHOR_POINT_TRAVERSE_ROTATE_K = .2;
const float ANCHOR_POINT_TRAVERSE_FORWARD_K = .3;
const float GO_HOME_ALIGN_K = .2;
const float GO_HOME_FORWARD_K = .3;
const float RESOURCES_COVERING_HOME_ALIGN_K = .3;
const float RESOURCES_COVERING_HOME_FORWARD_K = .4;
const float RESOURCES_COVERING_HOME_BLIND_K = .05;
const float RESOURCES_COVERING_HOME_BACKUP_K = .1;
const float GO_HOME_TRAVERSE_ROTATE_K = .1;
const float GO_HOME_TRAVERSE_FORWARD_K = .3;
const float DROPOFF_ALIGN_K = .2;
const float DROPOFF_FORWARD_K = .4;
const float DROPOFF_BACKUP_K = .1;
const float CLUSTER_ALIGN_K = .2;
const float CLUSTER_FORWARD_K = .25;
const float SEARCH_HOME_ROTATE_K = .1;
const float SEARCH_HOME_FORWARD_K = .1;

const float OBS_THRESHOLD = .6;
const float PICKUP_ALIGN_THRESHOLD = .03;
const float PICKUP_DISTANCE_THRESHOLD = .01;
const float PICKUP_BLIND_THRESHOLD = .02;
const float PICKUP_BACKUP_THRESHOLD = .05;
const float ROTATE_90_THRESHOLD = .1;
const float SONAR_RESOURCE_RANGE_THRESHOLD = .1;
const float ANCHOR_POINT_ALIGN_THRESHOLD = .2;
const float ANCHOR_POINT_FORWARD_THRESHOLD = .2;
const float ANCHOR_POINT_TRAVERSE_ROTATE_THRESHOLD = .2;
const float ANCHOR_POINT_TRAVERSE_FORWARD_THRESHOLD = .1;
const float GO_HOME_ALIGN_THRESHOLD = .2;
const float GO_HOME_FORWARD_THRESHOLD = .2;
const float RESOURCES_COVERING_HOME_ALIGN_THRESHOLD = .1;
const float RESOURCES_COVERING_HOME_THRESHOLD = .1;
const float RESOURCES_COVERING_HOME_BLIND_THRESHOLD = .1;
const float RESOURCES_COVERING_HOME_BACKUP_THRESHOLD = .1;
const float GO_HOME_TRAVERSE_ROTATE_THRESHOLD = .1;
const float GO_HOME_TRAVERSE_FORWARD_THRESHOLD = .1;
const float DROPOFF_ALIGN_THRESHOLD = .1;
const float DROPOFF_FORWARD_THRESHOLD = .01;
const float CLUSTER_ALIGN_THRESHOLD = .2;
const float SEARCH_HOME_ROTATE_THRESHOLD = .007;

float TRIGGER_DISTANCE = 0.6;

Tag average_home_tag;

const int TRAVERSAL_ROTATION_DEGREES = 20;
const float TRAVERSE_FORWARD_DISTANCE = .5;
const float HOME_FIELD_DISTANCE = 1.0;
const float RECRUITMENT_THRESHOLD = 7;

float home_tag_yaw_error = 0.0;
float resource_tag_yaw_error = 0.0;
float resource_tag_yaw_error_not_held = 0.0;
float distance_to_resource_tag_not_held = 0.0;
float distance_to_home_tag = 0.0;
float distance_to_resource_tag = 0.0;

int gripper_counter = 0;
int no_resource_tags_counter = 0;
int escaping_home_counter = 0;
int search_home_forward_counter = 2;
int search_wait_counter = 0;
int resource_held_counter = 0;
int resource_check_wait_counter = 10;
int wrist_raise_counter = 10;

const int MAX_TAG_COUNT = 16;
const int NO_TAGS_COUNTER_THRESHOLD = 20;
const int PICK_UP_BREAKOUT_THRESHOLD = 900;
const int ANCHOR_POINT_BREAKOUT_THRESHOLD = 3000;
const int DROP_OFF_BREAKOUT_THRESHOLD = 300;
const int CLUSTER_BREAKOUT_THRESHOLD = 3000;

int breakout_counter = 0;

#define NONE 0
#define LEFT 1
#define CENTER 2
#define RIGHT 3
int obs_side = NONE;
int home_tag_side = NONE;

float desired_heading = 0.0;
float desired_distance = 0.0;
float current_heading = 0.0;
float distance_travelled = 0.0;

Point pickup_goal_location;
Point resource_covering_home;
Point cluster_location;
Point anchor_location;
Point home_goal_location;
Point traverse_goal_location;
Point dropoff_goal_location;
Point dropoff_backup_location;
Point starting_drive_location;

vector<Point> anchor_points;
vector<Point> extra_anchor_points;
vector<Point> claimed_anchor_points;
Point north_anchor_point;
Point south_anchor_point;
Point east_anchor_point;
Point west_anchor_point;
Point NE_anchor_point;
Point NW_anchor_point;
Point SE_anchor_point;
Point SW_anchor_point;

float center_sonar_distance = 3;
float center_resource_distance = 3;
float center_resource_tag_not_held_distance = 3;
int count_resource_tags = 0;

string last_state = "";

// TODO: Refactor names below this line to match surf code.
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
  april_tag_subscriber = mNH.subscribe((published_name + "/targets"), 10, AprilTagHandler);					//receives tag data
  odometry_subscriber = mNH.subscribe((published_name + "/odom/filtered"), 10, OdometryHandler);				//receives ODOM data
  ekf_subscriber = mNH.subscribe((published_name + "/odom/ekf"), 10, EKFHandler);						//receives GPS data
  //virtualFenceSubscriber = mNH.subscribe(("/virtualFence"), 10, virtualFenceHandler);					//receives data for vitrual boundaries
  //manualWaypointSubscriber = mNH.subscribe((published_name + "/waypoints/cmd"), 10, manualWaypointHandler);		//receives manual waypoints given from GUI
  message_filters::Subscriber<sensor_msgs::Range> sonarLeftSubscriber(mNH, (published_name + "/sonarLeft"), 10);
  message_filters::Subscriber<sensor_msgs::Range> sonarCenterSubscriber(mNH, (published_name + "/sonarCenter"), 10);
  message_filters::Subscriber<sensor_msgs::Range> sonarRightSubscriber(mNH, (published_name + "/sonarRight"), 10);
  //recruitment_subscriber = mNH.subscribe("/detectionLocations", 10, RecruitmentHandler);

  //publishers
  state_machine_publisher = mNH.advertise<std_msgs::String>((published_name + "/state_machine"), 1, true);			//publishes state machine status
  status_publisher = mNH.advertise<std_msgs::String>((published_name + "/swarmie_status"), 1, true);			//publishes rover status
  finger_angle_publisher = mNH.advertise<std_msgs::Float32>((published_name + "/fingerAngle/cmd"), 1, true);			//publishes gripper angle to move gripper finger
  wrist_angle_publisher = mNH.advertise<std_msgs::Float32>((published_name + "/wristAngle/cmd"), 1, true);			//publishes wrist angle to move wrist
  drive_control_publisher = mNH.advertise<swarmie_msgs::Skid>((published_name + "/driveControl"), 10);			//publishes motor commands to the motors
  heartbeat_publisher = mNH.advertise<std_msgs::String>((published_name + "/behaviour/heartbeat"), 1, true);		//publishes ROSAdapters status via its "heartbeat"
  // TODO: What are obstaclePublisher and infoLogPublisher
  obstaclePublisher = mNH.advertise<std_msgs::UInt8>((published_name + "/obstacle"), 10, true);
  infoLogPublisher = mNH.advertise<std_msgs::String>("/infoLog", 1, true);						//publishes a message to the infolog box on GUI
  //waypointFeedbackPublisher = mNH.advertise<swarmie_msgs::Waypoint>((published_name + "/waypoints"), 1, true);		//publishes a waypoint to travel to if the rover is given a waypoint in manual mode

  //timers
  publish_status_timer = mNH.createTimer(ros::Duration(STATUS_PUBLISH_INTERVAL), PublishStatusTimerEventHandler);
  state_machine_timer = mNH.createTimer(ros::Duration(BEHAVIOR_LOOP_TIME_STEP), BehaviourStateMachine);
  publish_heartbeat_timer = mNH.createTimer(ros::Duration(HEARTBEAT_PUBLISH_INTERVAL), PublishHeartBeatTimerEventHandler);
  
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Range, sensor_msgs::Range, sensor_msgs::Range> sonarSyncPolicy;
  
  message_filters::Synchronizer<sonarSyncPolicy> sonarSync(sonarSyncPolicy(10), sonarLeftSubscriber, sonarCenterSubscriber, sonarRightSubscriber);
  sonarSync.registerCallback(boost::bind(&SonarHandler, _1, _2, _3));
  
  //transform_listener = new tf::TransformListener();

  std_msgs::String msg;
  msg.data = "Log Started";
  infoLogPublisher.publish(msg);
  stringstream ss;
  ss << "Rover start delay set to " << start_delay_in_seconds << " seconds";
  msg.data = ss.str();
  infoLogPublisher.publish(msg);

  /*if(current_mode != 2 && current_mode != 3)
  {
    // ensure the logic controller starts in the correct mode.
    logicController.SetModeManual();
  }*/

  timer_start_time = time(0);
  
  ros::spin();

  delete positionPublisher;
  return EXIT_SUCCESS;
}


// This is the top-most logic control block organised as a state machine.
// This function calls the dropOff, pickUp, and search controllers.
// This block passes the goal location to the proportional-integral-derivative
// controllers in the abridge package.
void BehaviourStateMachine(const ros::TimerEvent&)
{

  std_msgs::String state_machine_message;
  
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
      location_offset_odom.x = -1.3 * cos(current_location_odom.theta);
      location_offset_odom.y = -1.3 * sin(current_location_odom.theta);
      location_offset_ekf.x = -1.3 * cos(current_location_ekf.theta);
      location_offset_ekf.y = -1.3 * sin(current_location_ekf.theta);

      home_goal_location.x = 0.0;
      home_goal_location.y = 0.0;
      
      start_time = GetROSTimeInMilliSecs();
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
    
    switch(state_machine_state) {
            case INIT:
            {
                state_machine_message.data = "INIT";
                
                // initialize all points to 0
                resource_covering_home.x = 0.0;
                resource_covering_home.y = 0.0;
                pickup_goal_location.x = 0.0;
                pickup_goal_location.y = 0.0;
                cluster_location.x = 0.0;
                cluster_location.y = 0.0;
                anchor_location.x = 0.0;
                anchor_location.y = 0.0;
                traverse_goal_location.x = 0.0;
                traverse_goal_location.y = 0.0;
                home_goal_location.x = 0.0;
                home_goal_location.y = 0.0;
                dropoff_goal_location.x = 0.0;
                dropoff_goal_location.y = 0.0;
                dropoff_backup_location.x = 0.0;
                dropoff_backup_location.y = 0.0;
                starting_drive_location.x = 0.0;
                starting_drive_location.y = 0.0;

                linear_velocity = 0.0;
                angular_velocity = 0.0;
                obstacle_encountered = false;
                home_encountered = false;
                resource_encountered = false;
                resource_held = false;
                distance_to_resource_tag = 0.0;
                distance_to_home_tag = 0.0;
                inside_home = false;
                SEARCH_K = 1;


                vector<Point> claimed_anchor_points;

                breakout_counter = 0;
               
                raiseWrist();
                openFingers();
                state_machine_state = SEARCH;

                break;
            }
            case SEARCH:
            {
                state_machine_message.data = "SEARCH";
                linear_velocity = 0.1*SEARCH_K;
                angular_velocity = 0.0;
                

                //exit conditions
                if (!inside_home)
                {
                    if (home_encountered)
                    {
                        linear_velocity = 0.0;
                        angular_velocity = 0.0;
                        desired_heading = GetNewHeading(home_tag_side);
                        state_machine_state = SEARCH_OBSTACLE_AVOIDANCE;
                    }
                    else if (obstacle_encountered)
                    {
                        linear_velocity = 0.0;
                        angular_velocity = 0.0;
                        desired_heading = GetNewHeading(obs_side);
                        state_machine_state = SEARCH_OBSTACLE_AVOIDANCE;
                    }
                    else if (resource_encountered && hypot(current_location_odom.x - home_goal_location.x, current_location_odom.y - home_goal_location.y) > .55)
                    {
                        //desired alignment heading is set in AprilTagHandler "resource_tag_yaw_error"
                        linear_velocity = 0.0;
                        angular_velocity = 0.0;
                        breakout_counter = 0;
                        state_machine_state = PICKUP_ALIGN;             

                    }
                }
                if (escaping_home_counter >= 58)
                {
                    escaping_home_counter = 0;
                    SEARCH_K = 1;
                    home_encountered = false;
                    inside_home = false;
                }
                else if (inside_home)
                {
                    escaping_home_counter++;
                }
		        break;
            }
            case SEARCH_OBSTACLE_AVOIDANCE:
            {
                state_machine_message.data = "SEARCH_OBSTACLE_AVOIDANCE";
                if (fabs(desired_heading - current_location_odom.theta) < OBS_THRESHOLD)
                {
                    //reset detection sides
                    obs_side = NONE;
                    home_tag_side = NONE;

                    home_encountered = false;
                    resource_encountered = false;
                    obstacle_encountered = false;
                    linear_velocity = 0.0;
                    angular_velocity = 0.0;
                    state_machine_state = SEARCH;
                }
                else
                {
                    linear_velocity = 0.0;
                    angular_velocity = OBS_K * Wrap(desired_heading - current_location_odom.theta); // k*error
                }
                break;
            }
            case PICKUP_ALIGN:
            {
                state_machine_message.data = "PICKUP_ALIGN";
                if (home_encountered)
                {
                    linear_velocity = 0.0;
                    angular_velocity = 0.0;
                    desired_heading = GetNewHeading(home_tag_side);
                    breakout_counter = 0;
                    state_machine_state = SEARCH_OBSTACLE_AVOIDANCE; 
                }
                else if (obstacle_encountered)
                {
                    linear_velocity = 0.0;
                    angular_velocity = 0.0;
                    desired_heading = GetNewHeading(obs_side);
                    breakout_counter = 0;
                    state_machine_state = SEARCH_OBSTACLE_AVOIDANCE;
                }
                else if (fabs(resource_tag_yaw_error) < PICKUP_ALIGN_THRESHOLD) {
                    openFingers();
                    lowerWrist();
                    cout << "aligned" << endl;

                    linear_velocity = 0.0;
                    angular_velocity = 0.0;
                    
                    state_machine_state = PICKUP_FORWARD;

                    if (no_resource_tags_counter > NO_TAGS_COUNTER_THRESHOLD)
                    {
                        resource_encountered = false;
                        cout << "aligned but LOST THE CUBE ROTATE 90" << endl;
                        if ((resource_tag_yaw_error / fabs(resource_tag_yaw_error)) == 1) {
                            desired_heading = Wrap(M_PI/2 + current_location_odom.theta);
                        }
                        else if ((resource_tag_yaw_error / fabs(resource_tag_yaw_error)) == -1) {
                            desired_heading = Wrap(M_PI/2 - current_location_odom.theta);
                        }
                        state_machine_state = PICKUP_ROTATE_90;
                    }
                }
                else {
                    if (no_resource_tags_counter > NO_TAGS_COUNTER_THRESHOLD)
                    {
                        resource_encountered = false;
                        cout << "while aligning LOST THE CUBE ROTATE 90" << endl;
                        if ((resource_tag_yaw_error / fabs(resource_tag_yaw_error)) == 1) {
                            desired_heading = Wrap(M_PI/2 + current_location_odom.theta);
                        }
                        else if ((resource_tag_yaw_error / fabs(resource_tag_yaw_error)) == -1) {
                            desired_heading = Wrap(M_PI/2 - current_location_odom.theta);
                        }
                        state_machine_state = PICKUP_ROTATE_90;
                    }
                    linear_velocity = 0.0;
                    angular_velocity = -PICKUP_ALIGN_K * (resource_tag_yaw_error); //if pos k, will turn opposite direction
                }  
                if(PickupBreakout()) {
                    ResetFlags();
                    state_machine_state = SEARCH;
                }
                break;
            }
            case PICKUP_ROTATE_90:
            {
                state_machine_message.data = "PICKUP_ROTATE_90";
                linear_velocity = 0.0;
                if (home_encountered)
                {
                    linear_velocity = 0.0;
                    angular_velocity = 0.0;
                    desired_heading = GetNewHeading(home_tag_side);
                    breakout_counter = 0;
                    state_machine_state = SEARCH_OBSTACLE_AVOIDANCE;                     
                }
                else if (obstacle_encountered)
                {
                    linear_velocity = 0.0;
                    angular_velocity = 0.0;
                    desired_heading = GetNewHeading(obs_side);
                    breakout_counter = 0;
                    state_machine_state = SEARCH_OBSTACLE_AVOIDANCE;
                }
                else if (resource_encountered)
                {
                    //desired alignment heading is set in AprilTagHandler "resource_tag_yaw_error"
                    linear_velocity = 0.0;
                    angular_velocity = 0.0;
                    state_machine_state = PICKUP_ALIGN;
                }
                else {
                    if (fabs(desired_heading - current_location_odom.theta < ROTATE_90_THRESHOLD))
                    {
                        cout << "rotated 90 and saw no cube" << endl;
                        resource_encountered = false;
                        angular_velocity = 0.0;
                        breakout_counter = 0;
                        state_machine_state = SEARCH;
                    }
                    else
                    {
                        angular_velocity = ROTATE_90_K * Wrap(desired_heading - current_location_odom.theta);
                    }
                }
                if(PickupBreakout()){
                    ResetFlags();
                    state_machine_state = SEARCH;
                }
                break;
            }
            case PICKUP_FORWARD:
            {
                state_machine_message.data = "PICKUP_FORWARD";
                TRIGGER_DISTANCE = .3;
                if (hypot(current_location_odom.x - home_goal_location.x, current_location_odom.y - home_goal_location.y) < HOME_FIELD_DISTANCE)
                {
                    linear_velocity = 0.0;
                    angular_velocity = 0.0;
                    desired_heading = GetNewHeading(CENTER);
                    breakout_counter = 0;
                    state_machine_state = SEARCH_OBSTACLE_AVOIDANCE;
                }
                else if (home_encountered)
                {
                    resource_encountered = false;
                    linear_velocity = 0.0;
                    angular_velocity = 0.0;
                    TRIGGER_DISTANCE = .6; // set sonar distance back
                    desired_heading = GetNewHeading(home_tag_side);
                    breakout_counter = 0;
                    state_machine_state = SEARCH_OBSTACLE_AVOIDANCE; 
                }
                else if (obstacle_encountered)
                {
                    resource_encountered = false;
                    linear_velocity = 0.0;
                    angular_velocity = 0.0;
                    TRIGGER_DISTANCE = .6; // set sonar distance back
                    desired_heading = GetNewHeading(obs_side);
                    breakout_counter = 0;
                    state_machine_state = SEARCH_OBSTACLE_AVOIDANCE;
                }
                else if (distance_to_resource_tag < PICKUP_DISTANCE_THRESHOLD) { //firedrill; stop moving, drop claw, roll forward
                    cout << "distance travelled going to blind driving" << endl;
                    linear_velocity = 0.0;
                    angular_velocity = 0.0;
                    pickup_goal_location.x = current_location_odom.x + (distance_to_resource_tag + .09) * cos(current_location_odom.theta);
                    pickup_goal_location.y = current_location_odom.y + (distance_to_resource_tag + .09) * sin(current_location_odom.theta);
                    desired_distance = hypot(current_location_odom.x - pickup_goal_location.x,current_location_odom.y - pickup_goal_location.y); //needs to backup a little more
                    starting_drive_location.x = current_location_odom.x;
                    starting_drive_location.y = current_location_odom.y;
                    //still need to set sonar back but do it after blind drive and after closing fingers
                    state_machine_state = PICKUP_BLIND_DRIVE;
                }
                else {
                    linear_velocity = PICKUP_FORWARD_K * (distance_to_resource_tag);
                    angular_velocity =  -PICKUP_ALIGN_K * (resource_tag_yaw_error); //if pos k, will turn opposite direction
                }
                if(PickupBreakout()){
                    ResetFlags();
                    state_machine_state = SEARCH;
                }
                break;
            }
            case PICKUP_BLIND_DRIVE:
            {
                state_machine_message.data = "PICKUP_BLIND_DRIVE";
                //if (hypot(current_location_odom.x-pickup_goal_location.x,current_location_odom.y-pickup_goal_location.y) < PICKUP_BLIND_THRESHOLD) {
                if (CheckDriveDistance(desired_distance - PICKUP_BLIND_THRESHOLD)) {
                    cout << "blind drive is done close gripper " << endl;
                    linear_velocity = 0.0;
                    angular_velocity = 0.0;
                    state_machine_state = PICKUP_CLOSE_GRIPPER;
                }
                else {
                    linear_velocity = PICKUP_BLIND_K * hypot(current_location_odom.x-pickup_goal_location.x,current_location_odom.y-pickup_goal_location.y);
                    angular_velocity = 0.0;
                }
                if(PickupBreakout()){
                    ResetFlags();
                    state_machine_state = SEARCH;
                }
                break;
            }
            case PICKUP_CLOSE_GRIPPER:
            {
                state_machine_message.data = "PICKUP_CLOSE_GRIPPER";
                closeFingers();
                TRIGGER_DISTANCE = .6; // set sonar distance back
                
                if (gripper_counter == 10) {
                    starting_drive_location.x = current_location_odom.x;
                    starting_drive_location.y = current_location_odom.y;

                    if (RECRUITMENT) {
                        geometry_msgs::Pose2D temp_resource_location;
                        temp_resource_location.x = current_location_odom.x + (.15) * cos(current_location_odom.theta);
                        temp_resource_location.y = current_location_odom.y + (.15) * sin(current_location_odom.theta);
                        temp_resource_location.theta = current_location_odom.theta;
                        cout << "publishing resource location " << temp_resource_location.x << ", " << temp_resource_location.y << endl;
                        recruitment_publisher.publish(temp_resource_location);
                    }
                    state_machine_state = PICKUP_BACKUP_RAISE_GRIPPER;
                }
                else {
                    gripper_counter++;
                }
                cluster_location.x = current_location_odom.x;
                cluster_location.y = current_location_odom.y;
                if(PickupBreakout()){
                    ResetFlags();
                    state_machine_state = SEARCH;
                }
                break;
            }
            case PICKUP_BACKUP_RAISE_GRIPPER:
            {
                //back up distance needs to be increased
                state_machine_message.data = "PICKUP_BACKUP_RAISE_GRIPPER";
                raiseWrist();
                //if (desired_distance - hypot(current_location_odom.x-cluster_location.x,current_location_odom.y-cluster_location.y) < PICKUP_BACKUP_THRESHOLD) {
                if (CheckDriveDistance(desired_distance - PICKUP_BACKUP_THRESHOLD)) {  
                    linear_velocity = 0.0;
                    angular_velocity = 0.0;
                    desired_distance = 0.0;
                    state_machine_state = PICKUP_WAIT_TO_RAISE_GRIPPER;
                }
                else {
                    linear_velocity = -PICKUP_BACKUP_K * (desired_distance - hypot(current_location_odom.x-cluster_location.x,current_location_odom.y-cluster_location.y));
                    angular_velocity = 0.0;
                }
                if(PickupBreakout()){
                    ResetFlags();
                    state_machine_state = SEARCH;
                }
                break;
            }
            case PICKUP_WAIT_TO_RAISE_GRIPPER:
            {
                state_machine_message.data = "PICKUP_WAIT_TO_RAISE_GRIPPER";
                //check for cube here
                if (gripper_counter == 0) {
                    cout << "done waiting for gripper to raise" << endl;
                    if (resource_held) {
                        cout << "ET phone home" << endl;
                        anchor_location = ClosestAnchorPoint();
                        geometry_msgs::Pose2D temp_anchor;
                        temp_anchor.x = anchor_location.x;
                        temp_anchor.y = anchor_location.y;
                        temp_anchor.theta = anchor_location.theta;
                        //claim_anchor_point_publisher.publish(temp_anchor);
                        desired_heading = atan2(anchor_location.y - current_location_ekf.y, anchor_location.x - current_location_ekf.x);
                        breakout_counter = 0;
                        state_machine_state = ANCHOR_POINT_ALIGN;
                    }
                    else if (no_resource_tags_counter == 0) { //not holding but there are tags in view
                        cout << "done waiting not holding a cube, but still see a tag" << endl;
                        resource_encountered = false;
                        state_machine_state = PICKUP_ALIGN;
                    }
                    else { // no tags seen in view and not holding
                        cout << "done waiting not holding a cube, dont see any tags" << endl;
                        resource_encountered = false;
                        breakout_counter = 0;
                        state_machine_state = SEARCH;
                    }
                }
                else {
                    gripper_counter--;
                    if (center_resource_distance < .0015 && center_resource_distance != 0 || center_sonar_distance < SONAR_RESOURCE_RANGE_THRESHOLD) {
                        resource_held = true;
                    }
                }
                if(PickupBreakout()){
                    ResetFlags();
                    state_machine_state = SEARCH;
                }
                break;
            }
            case ANCHOR_POINT_ALIGN:
            {
                state_machine_message.data = "ANCHOR_POINT_ALIGN";
                linear_velocity = 0.0;
                if (home_encountered) {
                    cout << "encountered home during align" << endl;
                    angular_velocity = 0.0;
                    desired_heading = atan2(home_goal_location.y - current_location_odom.y, home_goal_location.x - current_location_odom.x);
                    breakout_counter = 0;
                    state_machine_state = DROPOFF_ALIGN;
                    // desired heading will be set by AprilTagHandler, home_tag_yaw_error
                }
                else if (fabs(desired_heading - current_location_ekf.theta) < ANCHOR_POINT_ALIGN_THRESHOLD) {
                    angular_velocity = 0.0;
                    cout << "aligned to anchorpoint" << endl;
                    state_machine_state = RESOURCE_HELD_CHECK;
                }
                else {
                    angular_velocity = ANCHOR_POINT_ALIGN_K * Wrap(desired_heading - current_location_odom.theta);                   
                }
                if(AnchorPointBreakout()){
                    ResetFlags();
                    state_machine_state = SEARCH_HOME;
                }
                break;
            }
            case ANCHOR_POINT_FORWARD:
            {
                state_machine_message.data = "ANCHOR_POINT_FORWARD";
                centerWrist();
                if (obstacle_encountered) {
                    linear_velocity = 0.0;
                    angular_velocity = 0.0;
                    desired_heading = Wrap(current_location_odom.theta + ((M_PI/180)*TRAVERSAL_ROTATION_DEGREES));
                    cout << "encountered an obstacle or tag, rotating 20 deg" << desired_heading << endl;
                    state_machine_state = ANCHOR_POINT_TRAVERSE_ROTATE;
                }
                else if (home_encountered) { //gets here when travelling home and you see home
                    linear_velocity = 0.0;
                    angular_velocity = 0.0;
                    cout << "encountered home during drive, ready to dropOff" << endl;
                    desired_heading = atan2(home_goal_location.y - current_location_odom.y, home_goal_location.x - current_location_odom.x);
                    breakout_counter = 0;
                    state_machine_state = DROPOFF_ALIGN;
                    //desired heading will be set by AprilTagHandler, home_tag_yaw_error
                }
                else if (resource_held_counter == 30) {
                    linear_velocity = 0.0;
                    angular_velocity = 0.0;
                    cout << "checking for resource held" << endl;
                    resource_held = false; // assume resource held false
                    state_machine_state = RESOURCE_HELD_CHECK;
                }
                else {
                    if (hypot(anchor_location.x - current_location_ekf.x, anchor_location.y - current_location_ekf.y) < ANCHOR_POINT_FORWARD_THRESHOLD) {
                        linear_velocity = 0.0;
                        angular_velocity = 0.0;
                        resource_held_counter = 0;
                        cout << "reached anchor point" << endl;
                        desired_heading = atan2(home_goal_location.y - current_location_odom.y, home_goal_location.x - current_location_odom.x);
                        breakout_counter = 0;
                        state_machine_state = GO_HOME_ALIGN;
                        //need to search for home here
                    }
                    else {
                        linear_velocity = ANCHOR_POINT_FORWARD_K * hypot(anchor_location.x - current_location_odom.x, anchor_location.y - current_location_odom.y);
                        angular_velocity = ANCHOR_POINT_ALIGN_K * Wrap(desired_heading - current_location_odom.theta);
                        resource_held_counter++;
                    }
                }
                if(AnchorPointBreakout()){
                    ResetFlags();
                    // geometry_msgs::Pose2D temp_anchor;
                    // temp_anchor.x = anchor_location.x;
                    // temp_anchor.y = anchor_location.y;
                    // temp_anchor.theta = anchor_location.theta;
                    // unclaim_anchor_point_publisher.publish(temp_anchor);
                    state_machine_state = SEARCH_HOME;
                }
                break;
            }
            case RESOURCE_HELD_CHECK:
            {
                state_machine_message.data = "RESOURCE_HELD_CHECK";
                raiseWrist();
                resource_held_counter = 0;
                if (resource_check_wait_counter == 0) {
                    resource_check_wait_counter = 10;
                    if (resource_held) {
                        centerWrist();
                        state_machine_state = ANCHOR_POINT_FORWARD;
                    }
                    else {
                        cout << "resource not held anymore" << endl;
                        resource_encountered = false;
                        openFingers();
                        desired_heading = atan2(cluster_location.y - current_location_odom.y, cluster_location.x - current_location_odom.x);
                        state_machine_state = CLUSTER_ALIGN;
                    }
                }
                else {
                    resource_check_wait_counter--;
                    if (center_resource_distance < .0015 && center_resource_distance != 0 || center_sonar_distance < SONAR_RESOURCE_RANGE_THRESHOLD) {
                        resource_held = true;
                    }
                }
                break;
            }
            case GO_HOME_ALIGN:
            {
                state_machine_message.data = "GO_HOME_ALIGN";
                linear_velocity = 0.0;
                if (home_encountered) {
                    cout << "encountered home during align" << endl;
                    angular_velocity = 0.0;
                    desired_heading = atan2(home_goal_location.y - current_location_odom.y, home_goal_location.x - current_location_odom.x);
                    state_machine_state = DROPOFF_ALIGN;
                }
                else if (fabs(desired_heading - current_location_odom.theta) < GO_HOME_ALIGN_THRESHOLD) {
                    angular_velocity = 0.0;
                    cout << "aligned to home" << endl;
                    
                    state_machine_state = GO_HOME_DRIVE;
                }
                else {
                    angular_velocity = GO_HOME_ALIGN_K * Wrap(desired_heading - current_location_odom.theta);
                }
                break;
            }
            case GO_HOME_DRIVE:
            {
                state_machine_message.data = "GO_HOME_DRIVE";

                if (obstacle_encountered) {
                    linear_velocity = 0.0;
                    angular_velocity = 0.0;
                    desired_heading = Wrap(current_location_odom.theta + ((M_PI/180)*TRAVERSAL_ROTATION_DEGREES));
                    cout << "encountered an obstacle or tag, rotating 20 deg" << desired_heading << endl;
                    state_machine_state = GO_HOME_TRAVERSE_ROTATE;
                }
                else if (home_encountered) {
                    linear_velocity = 0.0;
                    angular_velocity = 0.0;
                    cout << "encountered home during drive, ready to dropOff" << endl;
                    desired_heading = atan2(home_goal_location.y - current_location_odom.y, home_goal_location.x - current_location_odom.x);
                    state_machine_state = DROPOFF_ALIGN;
                    //desired heading will be set by AprilTagHandler, home_tag_yaw_error
                }
                else if (count_resource_tags >= MAX_TAG_COUNT) { //for testing needs to change
                    cout << "seen exactly " << count_resource_tags << "tags " << endl;
                    cout << "STOP seen a lot of cubes and no home tags " << endl;
                    linear_velocity = 0.0;
                    angular_velocity = 0.0;
                    state_machine_state = RESOURCES_COVERING_HOME_ALIGN;
                }
                else {
                    if (hypot(home_goal_location.x - current_location_ekf.x, home_goal_location.y - current_location_ekf.y) < GO_HOME_FORWARD_THRESHOLD) {
                        linear_velocity = 0.0;
                        angular_velocity = 0.0;
                        cout << "reached home point, don't see home" << endl;
                        lost_home = true;
                        desired_heading = Wrap(M_PI/2 + current_location_odom.theta);
                        state_machine_state = SEARCH_HOME_ROTATE;
                    }
                    else {
                        linear_velocity = GO_HOME_FORWARD_K * hypot(home_goal_location.x - current_location_odom.x, home_goal_location.y - current_location_odom.y);
                        angular_velocity = GO_HOME_ALIGN_K * Wrap(desired_heading - current_location_odom.theta);
                    }
                }
                break;
            }
            case RESOURCES_COVERING_HOME_ALIGN:
            {
                state_machine_message.data = "RESOURCES_COVERING_HOME_ALIGN";
                if (home_encountered)
                {
                    linear_velocity = 0.0;
                    angular_velocity = 0.0;
                    desired_heading = atan2(home_goal_location.y - current_location_odom.y, home_goal_location.x - current_location_odom.x);
                    
                    state_machine_state = DROPOFF_ALIGN; 
                }
                else if (fabs(resource_tag_yaw_error_not_held) < RESOURCES_COVERING_HOME_ALIGN_THRESHOLD) {
                    
                    cout << "aligned to stack RESOURCES_COVERING_HOME_ALIGN" << endl;
                    linear_velocity = 0.0;
                    angular_velocity = 0.0;
                    wrist_raise_counter = 10;
                    
                    state_machine_state = RESOURCES_COVERING_HOME_FORWARD;
                }
                else {
                    linear_velocity = 0.0;
                    angular_velocity = -RESOURCES_COVERING_HOME_ALIGN_K * (resource_tag_yaw_error_not_held); //if pos k, will turn opposite direction
                    if (no_resource_tags_counter > NO_TAGS_COUNTER_THRESHOLD)
                    { 
                        angular_velocity = 0.0;
                        state_machine_state = RESOURCES_COVERING_HOME_BACKUP;
                    }
                }
                break;
            }
            case RESOURCES_COVERING_HOME_FORWARD:
            {
                state_machine_message.data = "RESOURCES_COVERING_HOME_FORWARD";
                if (distance_to_resource_tag_not_held < RESOURCES_COVERING_HOME_THRESHOLD) { //firedrill; stop moving, drop claw, roll forward
                    linear_velocity = 0.0;
                    angular_velocity = 0.0;
                    resource_covering_home.x = current_location_odom.x + (distance_to_resource_tag_not_held + .08) * cos(current_location_odom.theta);
                    resource_covering_home.y = current_location_odom.y + (distance_to_resource_tag_not_held + .08) * sin(current_location_odom.theta);
                    desired_distance = hypot(current_location_odom.x-resource_covering_home.x,current_location_odom.y-resource_covering_home.y); //needs to backup a little more
                    starting_drive_location.x = current_location_odom.x;
                    starting_drive_location.y = current_location_odom.y;
                    cout << "Forward drive complete, blind driving" << endl;
                    if (wrist_raise_counter <= 0) {
                        wrist_raise_counter = 10;
                        state_machine_state = RESOURCES_COVERING_HOME_BLIND_DRIVE;
                    }
                    else {
                        raiseWrist();
                        wrist_raise_counter--;
                    }
                }
                else {
                    linear_velocity = RESOURCES_COVERING_HOME_FORWARD_K * (distance_to_resource_tag_not_held);
                    angular_velocity =  0.0; //if pos k, will turn opposite direction
                }
                break;
            }
            case RESOURCES_COVERING_HOME_BLIND_DRIVE:
            {
                state_machine_message.data = "RESOURCES_COVERING_BLIND_DRIVE";
                if (CheckDriveDistance(.08)) {
                    cout << "blind drive is done, open claw - then going to backup" << endl;
                    linear_velocity = 0.0;
                    angular_velocity = 0.0;
                    openFingers();
                    state_machine_state = RESOURCES_COVERING_HOME_BACKUP;
                }
                else {
                    linear_velocity = RESOURCES_COVERING_HOME_BLIND_K;// * hypot(current_location_odom.x-resource_covering_home.x,current_location_odom.y-resource_covering_home.y);
                    angular_velocity = 0.0;
                }
                break;
            }
            case RESOURCES_COVERING_HOME_BACKUP:
            {
                state_machine_message.data = "RESOURCES_COVERING_HOME_BACKUP";
                lowerWrist();
                if (CheckDriveDistance(.35)) {  
                    linear_velocity = 0.0;
                    angular_velocity = 0.0;
                    desired_distance = 0.0;
                    cout << "Back up complete, going to cluster align" << endl;
                    resource_held = false;
                    resource_encountered = false;
                    desired_heading = atan2(cluster_location.y - current_location_odom.y, cluster_location.x - current_location_odom.x);
                    state_machine_state = CLUSTER_ALIGN;
                }
                else {
                    linear_velocity = -RESOURCES_COVERING_HOME_BACKUP_K;
                    angular_velocity = 0.0;
                }
                break;
            }
            case SEARCH_HOME_ROTATE:
            {
                state_machine_message.data = "SEARCH_HOME_ROTATE";
                linear_velocity = 0.0;
                if (home_encountered)
                {
                    search_home_forward_counter = 0;
                    linear_velocity = 0.0;
                    angular_velocity = 0.0;
                    desired_heading = atan2(home_goal_location.y - current_location_odom.y, home_goal_location.x - current_location_odom.x);
                    state_machine_state = DROPOFF_ALIGN; 
                }
                else {
                    if (fabs(desired_heading - current_location_odom.theta) < SEARCH_HOME_ROTATE_THRESHOLD)
                    {
                        cout << "desired heading in search forward " << desired_heading << " current heading " << current_location_odom.theta << endl;
                        cout << "rotated 90 searching for home" << endl;
                        angular_velocity = 0.0;
                        desired_distance = search_home_forward_counter/2; 
                        cout << "desired distance " << desired_distance << "search_home_forward_counter " << search_home_forward_counter << endl;
                        starting_drive_location.x = current_location_odom.x;
                        starting_drive_location.y = current_location_odom.y;
                        state_machine_state = SEARCH_HOME_FORWARD;
                    }
                    else
                    {
                        angular_velocity = SEARCH_HOME_ROTATE_K * Wrap(desired_heading-current_location_odom.theta);
                        cout << "angular velocity in search home align " << angular_velocity << endl;
                        linear_velocity = 0.0;
                    }
                }
                break;
            }
            case SEARCH_HOME_FORWARD:
            {
                state_machine_message.data = "SEARCH_HOME_FORWARD";
                if (home_encountered) {
                    search_home_forward_counter = 2;
                    linear_velocity = 0.0;
                    angular_velocity = 0.0;
                    desired_heading = atan2(home_goal_location.y - current_location_odom.y, home_goal_location.x - current_location_odom.x);
                    state_machine_state = DROPOFF_ALIGN;
                }
                else if (obstacle_encountered) {
                    search_home_forward_counter = 2;
                    linear_velocity = 0.0;
                    angular_velocity = 0.0;
                    state_machine_state = SEARCH_HOME_OBSTACLE_WAIT;
                }
                else {
                    cout << "desired distance in forward " << desired_distance << " search_home_forward_counter " << search_home_forward_counter << endl;
                    if (CheckDriveDistance(desired_distance)) {
                        linear_velocity = 0.0;
                        angular_velocity = 0.0;
                        desired_heading = Wrap(M_PI/2 + current_location_odom.theta);
                        search_home_forward_counter++;
                        cout << "desired heading in search forward " << desired_heading << " current heading " << current_location_odom.theta << endl;
                        state_machine_state = SEARCH_HOME_ROTATE;
                    }
                    else {
                        linear_velocity = SEARCH_HOME_FORWARD_K;
                        angular_velocity = 0.0;
                    }
                }
                break;
            }
            case SEARCH_HOME_OBSTACLE_WAIT:
            {
                state_machine_message.data = "SEARCH_HOME_OBSTACLE_WAIT";
                if (search_wait_counter >= 100) {
                    search_wait_counter = 0;
                    linear_velocity = 0.0;
                    angular_velocity = 0.0;
                    state_machine_state = SEARCH_HOME;
                }
                else {
                    linear_velocity = 0.0;
                    angular_velocity = 0.0;
                    search_wait_counter++;
                }
                break;
            }
            case SEARCH_HOME:
            {
                state_machine_message.data = "SEARCH_HOME";
                linear_velocity = 0.1;
                angular_velocity = 0.0;
                if (home_encountered) {
                    linear_velocity = 0.0;
                    angular_velocity = 0.0;
                    desired_heading = atan2(home_goal_location.y - current_location_odom.y, home_goal_location.x - current_location_odom.x);
                    state_machine_state = DROPOFF_ALIGN;
                }
                else if (obstacle_encountered) {
                    linear_velocity = 0.0;
                    angular_velocity = 0.0;
                    desired_heading = GetNewHeading(obs_side);
                    state_machine_state = SEARCH_HOME_OBSTACLE_AVOIDANCE;
                }
                break;
            }
            case SEARCH_HOME_OBSTACLE_AVOIDANCE:
            {
                state_machine_message.data = "SEARCH_HOME_OBSTACLE_AVOIDANCE";
                if (fabs(desired_heading - current_location_odom.theta) < OBS_THRESHOLD)
                {
                    //reset detection sides
                    obs_side = NONE;
                    home_tag_side = NONE;

                    home_encountered = false;
                    obstacle_encountered = false;
                    linear_velocity = 0.0;
                    angular_velocity = 0.0;
                    state_machine_state = SEARCH_HOME;
                }
                else
                { 
                    linear_velocity = 0.0;
                    angular_velocity = OBS_K * Wrap(desired_heading - current_location_odom.theta); // k*error
                }
                break;
            }
            case ANCHOR_POINT_TRAVERSE_ROTATE:
            {
                state_machine_message.data = "ANCHOR_POINT_TRAVERSE_ROTATE";
                if (fabs(desired_heading - current_location_odom.theta) < ANCHOR_POINT_TRAVERSE_ROTATE_THRESHOLD) {
                    angular_velocity = 0.0;
                    linear_velocity = 0.0;
                    cout << "rotated 20 degrees" << endl;
                    desired_distance = TRAVERSE_FORWARD_DISTANCE;
                    traverse_goal_location.x = current_location_odom.x + desired_distance * cos(current_location_odom.theta);
                    traverse_goal_location.y = current_location_odom.y + desired_distance * sin(current_location_odom.theta);
                    obstacle_encountered = false;
                    state_machine_state = ANCHOR_POINT_TRAVERSE_FORWARD;
                }
                else {
                    angular_velocity = ANCHOR_POINT_TRAVERSE_ROTATE_K * Wrap(desired_heading - current_location_odom.theta);
                    linear_velocity = 0.0;
                }
                if(AnchorPointBreakout()){
                    ResetFlags();
                    // geometry_msgs::Pose2D temp_anchor;
                    // temp_anchor.x = anchor_location.x;
                    // temp_anchor.y = anchor_location.y;
                    // temp_anchor.theta = anchor_location.theta;
                    // unclaim_anchor_point_publisher.publish(temp_anchor);
                    state_machine_state = SEARCH_HOME;
                }
                break;
            }
            case ANCHOR_POINT_TRAVERSE_FORWARD:
            {
                state_machine_message.data = "ANCHOR_POINT_TRAVERSE_FORWARD";
                if (obstacle_encountered) {
                        angular_velocity = 0.0;
                        linear_velocity = 0.0;
                        desired_heading = Wrap(current_location_odom.theta + ((M_PI/180)*TRAVERSAL_ROTATION_DEGREES));
                        cout << "another obstacle in anchor point traverse forward" << endl;
                        state_machine_state = ANCHOR_POINT_TRAVERSE_ROTATE;
                }
                else if (hypot(traverse_goal_location.x - current_location_odom.x, traverse_goal_location.y - current_location_odom.y) < ANCHOR_POINT_TRAVERSE_FORWARD_THRESHOLD) {
                    angular_velocity = 0.0;
                    linear_velocity = 0.0;
                    desired_heading = atan2(anchor_location.y - current_location_ekf.y, anchor_location.x - current_location_ekf.x);
                    state_machine_state = ANCHOR_POINT_ALIGN;
                }
                else {
                    angular_velocity = 0.0;
                    linear_velocity = ANCHOR_POINT_TRAVERSE_FORWARD_K * (hypot(traverse_goal_location.x - current_location_odom.x, traverse_goal_location.y - current_location_odom.y)); 
                }
                if(AnchorPointBreakout()){
                    ResetFlags();
                    // geometry_msgs::Pose2D temp_anchor;
                    // temp_anchor.x = anchor_location.x;
                    // temp_anchor.y = anchor_location.y;
                    // temp_anchor.theta = anchor_location.theta;
                    // unclaim_anchor_point_publisher.publish(temp_anchor);
                    state_machine_state = SEARCH_HOME;
                }
                break;
            }
            case GO_HOME_TRAVERSE_ROTATE:
            {
                state_machine_message.data = "GO_HOME_TRAVERSE_ROTATE";
                if (fabs(desired_heading - current_location_odom.theta) < GO_HOME_TRAVERSE_ROTATE_THRESHOLD) {
                    angular_velocity = 0.0;
                    linear_velocity = 0.0;
                    desired_distance = TRAVERSE_FORWARD_DISTANCE;
                    traverse_goal_location.x = current_location_odom.x + desired_distance * cos(current_location_odom.theta);
                    traverse_goal_location.y = current_location_odom.y + desired_distance * sin(current_location_odom.theta);
                    obstacle_encountered = false;
                    
                    state_machine_state = GO_HOME_TRAVERSE_FORWARD;
                }
                else {
                    angular_velocity = GO_HOME_TRAVERSE_ROTATE_K * Wrap(desired_heading - current_location_odom.theta);
                    linear_velocity = 0.0;
                }
                break;
            }
            case GO_HOME_TRAVERSE_FORWARD:
            {
                state_machine_message.data = "GO_HOME_TRAVERSE_FORWARD";
                if (obstacle_encountered) {
                        angular_velocity = 0.0;
                        linear_velocity = 0.0;
                        desired_heading = Wrap(current_location_odom.theta + ((M_PI/180)*TRAVERSAL_ROTATION_DEGREES));
                        cout << "another obstacle in traverse forward" << endl;
                        state_machine_state = GO_HOME_TRAVERSE_ROTATE;
                }
                else if (hypot(traverse_goal_location.x - current_location_odom.x, traverse_goal_location.y - current_location_odom.y) < GO_HOME_TRAVERSE_FORWARD_THRESHOLD) {
                    angular_velocity = 0.0;
                    linear_velocity = 0.0;
                    desired_heading = atan2(anchor_location.y - current_location_ekf.y, anchor_location.x - current_location_ekf.x);
                    state_machine_state = ANCHOR_POINT_ALIGN;
                }
                else {
                    angular_velocity = 0.0;
                    linear_velocity = GO_HOME_TRAVERSE_FORWARD_K * (hypot(traverse_goal_location.x - current_location_odom.x, traverse_goal_location.y - current_location_odom.y)); 
                }
                break;
            }
            case DROPOFF_ALIGN:
            {
                state_machine_message.data = "DROPOFF_ALIGN";
                // we see home tag already
                linear_velocity = 0.0;
                if (fabs(average_home_tag.getPositionX()) < DROPOFF_ALIGN_THRESHOLD) {
                    angular_velocity = 0.0;
                    cout << "aligned to home from DROPOFF_ALIGN" << endl;
                    dropoff_goal_location.x = current_location_odom.x + (distance_to_home_tag + .12) * cos(current_location_odom.theta);
                    dropoff_goal_location.y = current_location_odom.y + (distance_to_home_tag + .12) * sin(current_location_odom.theta);
                    desired_distance = hypot(current_location_odom.x-dropoff_goal_location.x,current_location_odom.y-dropoff_goal_location.y) + .15;
                    dropoff_backup_location.x = current_location_odom.x;
                    dropoff_backup_location.y = current_location_odom.y;
                    raiseWrist();
                    starting_drive_location.x = current_location_odom.x;
                    starting_drive_location.y = current_location_odom.y;
                    state_machine_state = DROPOFF_FORWARD;
                }
                else {
                    angular_velocity = -DROPOFF_ALIGN_K * home_tag_yaw_error;
                }
                break;
            }
            case DROPOFF_FORWARD:
            {
                state_machine_message.data = "DROPOFF_FORWARD";
                //if (hypot(current_location_odom.x-dropoff_goal_location.x,current_location_odom.y-dropoff_goal_location.y) < .1) { //firedrill; stop moving, drop claw, roll forward
                if (CheckDriveDistance(desired_distance - .25)) {  
                    cout << "distance travelled, DROPOFF_FORWARD" << endl;
                    linear_velocity = 0.0;
                    angular_velocity = 0.0;
                    openFingers();
                    starting_drive_location.x = current_location_odom.x;
                    starting_drive_location.y = current_location_odom.y;
                    if (lost_home) {
                        SubtractLocationOffset();
                        cout << "current location x before open claws " << current_location_odom.x << endl;
                        location_offset_odom.x = -(0.5 * cos(current_location_odom.theta) + current_location_odom.x) + location_offset_odom.x;
                        location_offset_odom.y = -(0.5 * sin(current_location_odom.theta) + current_location_odom.y) + location_offset_odom.y;
                        location_offset_ekf.x = -(0.5 * cos(current_location_ekf.theta) + current_location_ekf.x) + location_offset_ekf.x;
                        location_offset_ekf.y = -(0.5 * sin(current_location_ekf.theta) + current_location_ekf.y) + location_offset_ekf.y;
                        cout << "current location x after open claws " << current_location_odom.x << endl;
                        cout << "location offset odom x " << location_offset_odom.x << "location offset ekf " << location_offset_ekf.x << endl;
                        AddLocationOffset();
                     }
                    lost_home = false;
                    cout << "desired_distance " << desired_distance << endl;
                    state_machine_state = DROPOFF_BACKUP_OPEN_GRIPPER;
                }
                else {
                    linear_velocity = DROPOFF_FORWARD_K * (hypot(current_location_odom.x-dropoff_goal_location.x,current_location_odom.y-dropoff_goal_location.y));
                    angular_velocity =  0.0; //-DROPOFF_ALIGN_K * (home_tag_yaw_error); //if pos k, will turn opposite direction
                }
                if(DropOffBreakout()){
                    openFingers();
                    //unclaim any anchor points
                    // geometry_msgs::Pose2D temp_anchor;
                    // temp_anchor.x = anchor_location.x;
                    // temp_anchor.y = anchor_location.y;
                    // temp_anchor.theta = anchor_location.theta;
                    // int index = ContainsClaimedAnchorPoints(temp_anchor);
                    // if (index != -1) {
                    //     cout << "anchor point x before unclaiming" << claimed_anchor_points.at(index).x << endl;
                    //     claimed_anchor_points.erase(claimed_anchor_points.begin() + index);
                    // }
                    state_machine_state = DROPOFF_BACKUP_OPEN_GRIPPER;
                }
                break;
            }
            case DROPOFF_BACKUP_OPEN_GRIPPER:
            {
                state_machine_message.data = "DROPOFF_BACKUP_OPEN_GRIPPER";
                raiseWrist();
                //if (desired_distance - hypot(current_location_odom.x-dropoff_backup_location.x,current_location_odom.y-dropoff_backup_location.y) < .1) {
                if (CheckDriveDistance(.35)){  
                    linear_velocity = 0.0;
                    angular_velocity = 0.0;
                    desired_distance = 0.0;
                    resource_held = false;
                    resource_encountered = false;
                    geometry_msgs::Pose2D temp_anchor;
                    temp_anchor.x = anchor_location.x;
                    temp_anchor.y = anchor_location.y;
                    temp_anchor.theta = anchor_location.theta;
                    //unclaim_anchor_point_publisher.publish(temp_anchor);
                    if (SITE_FIDELITY) {
                        desired_heading = atan2(cluster_location.y - current_location_odom.y, cluster_location.x - current_location_odom.x);
                        state_machine_state = CLUSTER_ALIGN;
                    }
                    else
                    {
                        state_machine_state = SEARCH;
                    }
                }
                else {
                    linear_velocity = -DROPOFF_BACKUP_K; //* (desired_distance - hypot(current_location_odom.x-dropoff_backup_location.x,current_location_odom.y-dropoff_backup_location.y));
                    angular_velocity = 0.0;
                }
                break;
            }
            case CLUSTER_ALIGN:
            {
                state_machine_message.data = "CLUSTER_ALIGN";
                
                resource_encountered = false;
                if (fabs(desired_heading - current_location_odom.theta) < CLUSTER_ALIGN_THRESHOLD) {
                    geometry_msgs::Pose2D temp_anchor;
                    temp_anchor.x = anchor_location.x;
                    temp_anchor.y = anchor_location.y;
                    temp_anchor.theta = anchor_location.theta;
                    //unclaim_anchor_point_publisher.publish(temp_anchor); //duplicated from backup, needs to be sure to unclaim if you drop a cube while going home
                    linear_velocity = 0.0;
                    angular_velocity = 0.0;
                    desired_distance = hypot(current_location_odom.x - cluster_location.x, current_location_odom.y - cluster_location.y);
                    home_encountered = false;
                    state_machine_state = CLUSTER_FORWARD; 
                }
                else {
                    angular_velocity = CLUSTER_ALIGN_K * Wrap(desired_heading - current_location_odom.theta);
                    linear_velocity = 0.0;
                }
                if (cluster_location.x == 0.0 || cluster_location.y == 0.0) {
                    linear_velocity = 0.0;
                    angular_velocity = 0.0;
                    home_encountered = false;
                    resource_encountered = false;
                    obstacle_encountered = false;
                    resource_held = false;
                    breakout_counter = 0;
                    state_machine_state = SEARCH;
                }
                if(ClusterBreakout()){
                    ResetFlags();
                    geometry_msgs::Pose2D temp_anchor;
                    temp_anchor.x = anchor_location.x;
                    temp_anchor.y = anchor_location.y;
                    temp_anchor.theta = anchor_location.theta;
                    //unclaim_anchor_point_publisher.publish(temp_anchor);
                    state_machine_state = SEARCH;
                }
                break;
            }
            case CLUSTER_FORWARD:
            {
                state_machine_message.data = "CLUSTER_FORWARD";
                if (home_encountered) {
                    linear_velocity = 0.0;
                    angular_velocity = 0.0;
                    desired_heading = Wrap(current_location_odom.theta + ((M_PI/180)*TRAVERSAL_ROTATION_DEGREES));
                    cout << "encountered home on way to cluster, rotating 20 deg" << desired_heading << endl;
                    state_machine_state = CLUSTER_TRAVERSE_ROTATE;
                }
                else if (obstacle_encountered) {
                    linear_velocity = 0.0;
                    angular_velocity = 0.0;
                    desired_heading = Wrap(current_location_odom.theta + ((M_PI/180)*TRAVERSAL_ROTATION_DEGREES));
                    cout << "encountered obstacle on way to cluster, rotating 20 deg" << desired_heading << endl;
                    state_machine_state = CLUSTER_TRAVERSE_ROTATE;
                }
                else if (resource_encountered) {
                    linear_velocity = 0.0;
                    angular_velocity - 0.0;
                    breakout_counter = 0;
                    state_machine_state = PICKUP_ALIGN;
                }
                else if (hypot(current_location_odom.x - cluster_location.x, current_location_odom.y - cluster_location.y) < .1) {
                    linear_velocity = 0.0;
                    angular_velocity = 0.0;
                    cout << "reached the cluster location" << endl;
                    //desired_heading = Wrap(M_PI - current_location_odom.theta);
                    //cluster_sweep_left = true;
                    //state_machine_state = CLUSTER_SWEEP;
                    breakout_counter = 0;
                    state_machine_state = SEARCH;
                }
                else {
                    linear_velocity = CLUSTER_FORWARD_K * hypot(cluster_location.x - current_location_odom.x, cluster_location.y - current_location_odom.y);
                    angular_velocity = CLUSTER_ALIGN_K * Wrap(desired_heading - current_location_odom.theta);
                }
                if(ClusterBreakout()){
                    ResetFlags();
                    state_machine_state = SEARCH;
                }
                break;
            }
            //case CLUSTER_SWEEP:
            //{
                // state_machine_message.data = "CLUSTER_SWEEP";
                // if (resource_encountered) {
                //     linear_velocity = 0.0;
                //     angular_velocity = 0.0;
                //     state_machine_state = PICKUP_ALIGN;
                // }
                // else if (cluster_sweep_left) {
                //     if (fabs(desired_heading - current_location_odom.theta < ROTATE_90_THRESHOLD)) {
                //         cluster_sweep_left = false;
                //         desired_heading = Wrap(M_PI + current_location_odom.theta);
                //         linear_velocity = 0.0;
                //         angular_velocity = 0.0;
                //     }
                //     else {
                //         angular_velocity = ROTATE_90_K * Wrap(desired_heading - current_location_odom.theta);
                //     }
                // }
                // else if (!cluster_sweep_left) {
                //     if (fabs(desired_heading - current_location_odom.theta < ROTATE_90_THRESHOLD)) {
                //         linear_velocity = 0.0;
                //         angular_velocity = 0.0;
                //         state_machine_state = SEARCH;
                //     }
                //     else {
                //         angular_velocity = ROTATE_90_K * Wrap(desired_heading - current_location_odom.theta);
                //     }
                // }
                // break;
            //}
            case CLUSTER_TRAVERSE_ROTATE:
            {
                state_machine_message.data = "CLUSTER_TRAVERSE_ROTATE";
                if (fabs(desired_heading - current_location_odom.theta) < ANCHOR_POINT_TRAVERSE_ROTATE_THRESHOLD) {
                    angular_velocity = 0.0;
                    linear_velocity = 0.0;
                    cout << "rotated 20 degrees" << endl;
                    desired_distance = TRAVERSE_FORWARD_DISTANCE;
                    // possibly rename this variable, also used in ANCHOR_POINT_TRAVERSE_ROTATE
                    traverse_goal_location.x = current_location_odom.x + desired_distance * cos(current_location_odom.theta);
                    traverse_goal_location.y = current_location_odom.y + desired_distance * sin(current_location_odom.theta);
                    obstacle_encountered = false;
                    home_encountered = false;
                    state_machine_state = CLUSTER_TRAVERSE_FORWARD;
                }
                else {
                    angular_velocity = .4 * Wrap(desired_heading - current_location_odom.theta);
                    linear_velocity = 0.0;
                }
                if(ClusterBreakout()){
                    ResetFlags();
                    state_machine_state = SEARCH;
                }
                break;
            }
            case CLUSTER_TRAVERSE_FORWARD:
            {
                state_machine_message.data = "CLUSTER_TRAVERSE_FORWARD";
                if (obstacle_encountered) {
                    angular_velocity = 0.0;
                    linear_velocity = 0.0;
                    desired_heading = Wrap(current_location_odom.theta + ((M_PI/180)*TRAVERSAL_ROTATION_DEGREES));
                    cout << "another obstacle in anchor point traverse forward" << endl;
                    state_machine_state = CLUSTER_TRAVERSE_ROTATE;
                }
                else if (home_encountered) {
                    angular_velocity = 0.0;
                    linear_velocity = 0.0;
                    desired_heading = Wrap(current_location_odom.theta + ((M_PI/180)*TRAVERSAL_ROTATION_DEGREES));
                    cout << "home was seen during traversal towards cluster_location" << endl;
                    state_machine_state = CLUSTER_TRAVERSE_ROTATE;
                }
                else if (hypot(traverse_goal_location.x - current_location_odom.x, traverse_goal_location.y - current_location_odom.y) < ANCHOR_POINT_TRAVERSE_FORWARD_THRESHOLD) {
                    angular_velocity = 0.0;
                    linear_velocity = 0.0;
                    desired_heading = atan2(cluster_location.y - current_location_ekf.y, cluster_location.x - current_location_ekf.x);
                    state_machine_state = CLUSTER_ALIGN;
                }
                else {
                    angular_velocity = 0.0;
                    linear_velocity = .3 * (hypot(traverse_goal_location.x - current_location_odom.x, traverse_goal_location.y - current_location_odom.y)); 
                }
                if(ClusterBreakout()){
                    ResetFlags();
                    state_machine_state = SEARCH;
                }
                break;
            }
            default:
            {
                state_machine_message.data = "DEFAULT HIT";
                break;
            }
        } /* end of switch() */
        ActuatorOutput(linear_velocity,angular_velocity);

    /*bool wait = false;	//a variable created to check if we are in a waiting state
    
    //do this when wait behaviour happens
    if (wait)
    {
      sendDriveCommand(0.0,0.0);
      std_msgs::Float32 angle;
      
      angle.data = prevFinger;
      finger_angle_publisher.publish(angle);
      angle.data = prevWrist;
      wrist_angle_publisher.publish(angle);
    }*/
    
    //collision_msg.data = logicController.getCollisionCalls();
    //obstaclePublisher.publish(collision_msg);
    //publishHandeling here
    //logicController.getPublishData(); //Not Currently Implemented, used to get data from logic controller and publish to the appropriate ROS Topic; Suggested
    
    
    //adds a blank space between sets of debugging data to easily tell one tick from the next
    //cout << endl;
    
  }
  
  // mode is NOT auto
  else	//manual mode
  {
    humanTime();

    //logicController.SetCurrentTimeInMilliSecs( GetROSTimeInMilliSecs() );

    // publish current state for the operator to see
    state_machine_message.data = "WAITING";

    //openFingers();
    state_machine_state = INIT;
  }

  // publish state machine string for user, only if it has changed, though
  state_machine_publisher.publish(state_machine_message);

  if (last_state.compare(state_machine_message.data) != 0)
  {
    last_state = state_machine_message.data;
    StateLogger((state_machine_message.data));
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

void AprilTagHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message) {

    float sum_x = 0.0;
    float sum_y = 0.0;
    float sum_z = 0.0;
    int count_home_tags = 0;
    count_resource_tags = 0;
    int in_home_tags = 0;
    int out_home_tags = 0;
    //Tag average_home_tag; //moved to global
    Tag chosen_tag;
    Tag chosen_tag_not_held;
    float distance_to_closest_resource_tag = std::numeric_limits<double>::max();
    float distance_to_closest_resource_tag_not_held = std::numeric_limits<double>::max();
    int index_of_closest_resource_tag = -1;
    int index_of_closest_resource_tag_not_held = -1;
    
    if (message->detections.size() > 0) {
        vector<Tag> home_tags;
        vector<Tag> resource_tags;
        vector<Tag> candidate_resource_tags;
        vector<Tag> candidate_resource_tags_not_held;
        for (int i = 0; i < message->detections.size(); i++) {

            // Package up the ROS AprilTag data into our own type that does not rely on ROS.
            Tag detected_tag;
            detected_tag.setID( message->detections[i].id );

            // Pass the position of the AprilTag
            geometry_msgs::PoseStamped tagPose = message->detections[i].pose;
            detected_tag.setPosition( make_tuple( tagPose.pose.position.x,
                                     tagPose.pose.position.y,
                                     tagPose.pose.position.z ) );

            // Pass the orientation of the AprilTag
            detected_tag.setOrientation( ::boost::math::quaternion<float>( tagPose.pose.orientation.x,
                                                              tagPose.pose.orientation.y,
                                                              tagPose.pose.orientation.z,
                                                              tagPose.pose.orientation.w ) );
        

            

            if (message->detections[i].id == 256) {
                count_home_tags++;
		        if (detected_tag.calcYaw() < 0)
            	{
                    in_home_tags++;
            	}
                else {
                    out_home_tags++;
                }

                sum_x = sum_x + detected_tag.getPositionX();
                sum_y = sum_y + detected_tag.getPositionY();
                sum_z = sum_z + detected_tag.getPositionZ();
                home_tags.push_back(detected_tag);
            }
            else
            {
                count_resource_tags++;
                double distant_to_detected_tag = hypot(hypot(detected_tag.getPositionX(), detected_tag.getPositionY()), detected_tag.getPositionZ());
                if(fabs(detected_tag.getPositionX())<0.25){
                    candidate_resource_tags.push_back(detected_tag);
                    if(detected_tag.getPositionY()<-.02){
                        candidate_resource_tags_not_held.push_back(detected_tag);
                    }
                }
                else if (distance_to_closest_resource_tag > distant_to_detected_tag){
                    index_of_closest_resource_tag = i;
                    distance_to_closest_resource_tag = distant_to_detected_tag;
                    if(detected_tag.getPositionY()<-.02){
                        distance_to_closest_resource_tag_not_held = distant_to_detected_tag;
                        index_of_closest_resource_tag_not_held = distant_to_detected_tag;
                    }
                }
                resource_tags.push_back(detected_tag);
            }
        } // for loop end

        if (in_home_tags > out_home_tags)
        {   
            SEARCH_K = -1;
            inside_home = true;
        }
        else
        {
            SEARCH_K = 1;
            inside_home = false;
        }

        if (count_home_tags > 0) {
            home_encountered = true;

            average_home_tag.setPositionX(sum_x / (count_home_tags));
            average_home_tag.setPositionY(sum_y / (count_home_tags));
            average_home_tag.setPositionZ(sum_z / (count_home_tags));

            float distance_to_home_tag_from_camera;

            home_tag_yaw_error = 0.0;
            // using a^2 + b^2 = c^2 to find the distance to the block
            // 0.195 is the height of the camera lens above the ground in cm.
            //
            // a is the linear distance from the robot to the block, c is the
            // distance from the camera lens, and b is the height of the
            // camera above the ground.
            distance_to_home_tag_from_camera = hypot(hypot(average_home_tag.getPositionX(), average_home_tag.getPositionY()),
                                            average_home_tag.getPositionZ());


            if ((distance_to_home_tag_from_camera * distance_to_home_tag_from_camera - CAMERA_HEIGHT_IN_CM * CAMERA_HEIGHT_IN_CM) > 0) {
                distance_to_home_tag = sqrt(distance_to_home_tag_from_camera * distance_to_home_tag_from_camera - CAMERA_HEIGHT_IN_CM * CAMERA_HEIGHT_IN_CM);
            } else {
                float epsilon = 0.00001; // A small non-zero positive number
                distance_to_home_tag = epsilon;
            }
            home_tag_yaw_error = atan((average_home_tag.getPositionX() + CAMERA_OFFSET_CORRECTION) / distance_to_home_tag) * 1.05; //angle to block from bottom center of chassis on the horizontal.

            if (home_tag_yaw_error < -0.1 ){
              home_tag_side = LEFT;
            }
            else if(home_tag_yaw_error >= -0.1 && home_tag_yaw_error <= 0.1){
              home_tag_side = CENTER;
            }
            else{
              home_tag_side = RIGHT;
            }
        }

        if (count_resource_tags > 0) {
            resource_encountered = true;

            if(candidate_resource_tags.size() > 0) {
                float closest_candidate_tag_distance = std::numeric_limits<double>::max(); 
                int target_idx = -1;
                //this loop selects the closest visible block that is near the center of the screen to makes goals for it
                for ( int i = 0; i < candidate_resource_tags.size(); i++ ) {
                    float current_tag_distance = hypot(hypot(candidate_resource_tags[i].getPositionX(), candidate_resource_tags[i].getPositionY()), candidate_resource_tags[i].getPositionZ());
                    if (closest_candidate_tag_distance > current_tag_distance)
                    {
                        target_idx = i;
                        closest_candidate_tag_distance = current_tag_distance;
                    }
                    chosen_tag = candidate_resource_tags[target_idx];
                }    
            } else {
                chosen_tag = resource_tags[index_of_closest_resource_tag];
            }

            //want tag that is close to center but not in gripper
            if(candidate_resource_tags_not_held.size() > 0) {
                float closest_candidate_tag_distance_not_held = std::numeric_limits<double>::max();
                int target_idx = -1;
                for ( int i = 0; i < candidate_resource_tags.size(); i++ ) {
                    float current_tag_distance = hypot(hypot(candidate_resource_tags_not_held[i].getPositionX(), candidate_resource_tags_not_held[i].getPositionY()), candidate_resource_tags_not_held[i].getPositionZ());
                    if (closest_candidate_tag_distance_not_held > current_tag_distance) {
                        target_idx = i;
                        closest_candidate_tag_distance_not_held = current_tag_distance;
                    }
                    chosen_tag_not_held = candidate_resource_tags_not_held[target_idx];
                }
            }
            else {
                chosen_tag_not_held = resource_tags[index_of_closest_resource_tag_not_held];
            }

            float distance_to_resource_tag_from_camera;
            float distance_to_resource_tag_from_camera_not_held;

            resource_tag_yaw_error = 0.0;
            // using a^2 + b^2 = c^2 to find the distance to the block
            // 0.195 is the height of the camera lens above the ground in cm.
            //
            // a is the linear distance from the robot to the block, c is the
            // distance from the camera lens, and b is the height of the
            // camera above the ground.
            distance_to_resource_tag_from_camera = hypot(hypot(chosen_tag.getPositionX(), chosen_tag.getPositionY()),
                                            chosen_tag.getPositionZ());

            if ((distance_to_resource_tag_from_camera * distance_to_resource_tag_from_camera - CAMERA_HEIGHT_IN_CM * CAMERA_HEIGHT_IN_CM) > 0) {
                distance_to_resource_tag = sqrt(distance_to_resource_tag_from_camera * distance_to_resource_tag_from_camera - CAMERA_HEIGHT_IN_CM * CAMERA_HEIGHT_IN_CM);
            } else {
                float epsilon = 0.00001; // A small non-zero positive number
                distance_to_resource_tag = epsilon;
            }
            resource_tag_yaw_error = CAMERA_OFFSET_CORRECTION + chosen_tag.getPositionX();
            center_resource_distance = distance_to_resource_tag;


            // lol wut


            distance_to_resource_tag_from_camera_not_held = hypot(hypot(chosen_tag.getPositionX(), chosen_tag.getPositionY()),
                                            chosen_tag.getPositionZ());

            if ((distance_to_resource_tag_from_camera_not_held * distance_to_resource_tag_from_camera_not_held - CAMERA_HEIGHT_IN_CM * CAMERA_HEIGHT_IN_CM) > 0) {
                distance_to_resource_tag_not_held = sqrt(distance_to_resource_tag_from_camera_not_held * distance_to_resource_tag_from_camera_not_held - CAMERA_HEIGHT_IN_CM * CAMERA_HEIGHT_IN_CM);
            } else {
                float epsilon = 0.00001; // A small non-zero positive number
                distance_to_resource_tag_not_held = epsilon;
            }
            resource_tag_yaw_error_not_held = CAMERA_OFFSET_CORRECTION + chosen_tag_not_held.getPositionX();
            no_resource_tags_counter = 0;
            
        }
    }
    else
    {
        //no tags in screen
        no_resource_tags_counter++;
        center_resource_distance = 0.0;
    }
}

void ModeHandler(const std_msgs::UInt8::ConstPtr& message) {
    current_mode = message->data;
    sendDriveCommand(0.0, 0.0);
}

void SonarHandler(const sensor_msgs::Range::ConstPtr& sonar_left, const sensor_msgs::Range::ConstPtr& sonar_center, const sensor_msgs::Range::ConstPtr& sonar_right) {

    //if any sonar is below the trigger distance set physical obstacle true

    if ( sonar_left->range < TRIGGER_DISTANCE || sonar_center->range < TRIGGER_DISTANCE || sonar_right->range < TRIGGER_DISTANCE ) {
        obstacle_encountered = true;
        if (sonar_left->range < TRIGGER_DISTANCE) {
            obs_side = LEFT;
        }
        else if (sonar_right->range < TRIGGER_DISTANCE) {
            obs_side = RIGHT;
        }
        else if (sonar_center->range < TRIGGER_DISTANCE) {
            if (sonar_center->range < SONAR_RESOURCE_RANGE_THRESHOLD) {
                center_sonar_distance = sonar_center->range;
            }
            obs_side = CENTER;
        }
    }
}

void OdometryHandler(const nav_msgs::Odometry::ConstPtr& message) {
    //Get (x,y) location directly from pose
    current_location_odom.x = message->pose.pose.position.x + location_offset_odom.x;
    current_location_odom.y = message->pose.pose.position.y + location_offset_odom.y;

      
    //Get theta rotation by converting quaternion orientation to pitch/roll/yaw
    tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_location_odom.theta = yaw; 
    if (SIMULATOR) {
        current_location_ekf.x = current_location_odom.x;
        current_location_ekf.y = current_location_odom.y;
        current_location_ekf.theta = current_location_odom.theta;
    }
}

// Allows a virtual fence to be defined and enabled or disabled through ROS
/*void virtualFenceHandler(const std_msgs::Float32MultiArray& message) 
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
}*/

void EKFHandler(const nav_msgs::Odometry::ConstPtr& message) {
    if (!SIMULATOR) {
        //Get (x,y) location directly from pose
        current_location_ekf.x = message->pose.pose.position.x + location_offset_ekf.x;
        current_location_ekf.y = message->pose.pose.position.y + location_offset_ekf.y;
        //Get theta rotation by converting quaternion orientation to pitch/roll/yaw
        tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_location_ekf.theta = yaw;
        // cout << published_name << ":  " << current_location_ekf.x << ", " << current_location_ekf.y << endl;
    }
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

        sendDriveCommand(left, right); //add some fraction to make manual mode slower/faster
    }
}


void PublishStatusTimerEventHandler(const ros::TimerEvent&) {
  std_msgs::String msg;
  msg.data = "online";		//change this with team name
  status_publisher.publish(msg);
}

/*void manualWaypointHandler(const swarmie_msgs::Waypoint& message) {
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
}*/

void RecruitmentHandler(const swarmie_msgs::Recruitment& msg)
{
   if(msg.name.data != published_name) {
      Point p;
      p.x = msg.x;
      p.y = msg.y;
      //logicController.gotRecruitmentMessage(p);
   }
}

void SigintEventHandler(int sig) {
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

void PublishHeartBeatTimerEventHandler(const ros::TimerEvent&) {
  std_msgs::String msg;
  msg.data = "";
  heartbeat_publisher.publish(msg);
}

long int GetROSTimeInMilliSecs()
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
  
  float timeDiff = (GetROSTimeInMilliSecs()-start_time)/1e3;
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

/////////////////////////////
// START OF SURF 2018 FUNCTION
/////////////////////////////

void ActuatorOutput(float linear_velocity_out, float angular_velocity_out) {
    //prevent combine output from going over tihs value
    const int SATURATION_VALUE = 180;
    const float MAX_LINEAR_VELOCITY = 0.3;
    const float MAX_ANGULAR_VELOCITY = 0.3;

    //cout << "BEFORE linear: " << linear_velocity_out << " BEFORE angular: " << angular_velocity_out << endl;

    if(fabs(linear_velocity_out) + fabs(angular_velocity_out) > MAX_ANGULAR_VELOCITY){
        linear_velocity_out = std::max(0.0, (double)(MAX_ANGULAR_VELOCITY - fabs(angular_velocity_out)));
        if(angular_velocity_out != 0.0){
            angular_velocity_out = (angular_velocity_out/fabs(angular_velocity_out)) * (MAX_ANGULAR_VELOCITY - linear_velocity_out);
        }
    }

    //cout << "AFTER linear: " << linear_velocity_out << " AFTER angular: " << angular_velocity_out << endl;

    if (linear_velocity_out > 0 && linear_velocity_out < .01) {
        linear_velocity_out = .01;
    }
    if (linear_velocity_out < 0 && linear_velocity_out > -.01) {
        linear_velocity_out = -.01;
    }
    if (angular_velocity_out > 0 && angular_velocity_out < .008) {
        angular_velocity_out = .008;
    }
    if (angular_velocity_out < 0 && angular_velocity_out > -.008) {
        angular_velocity_out = -.008;
    }

    linear_velocity_out = linear_velocity_out * (SATURATION_VALUE / MAX_LINEAR_VELOCITY);
    angular_velocity_out = angular_velocity_out * (SATURATION_VALUE / MAX_ANGULAR_VELOCITY );

    
    int left = linear_velocity_out - angular_velocity_out;
    int right = linear_velocity_out + angular_velocity_out;

    double left_velocity = SaturationCheck(left,SATURATION_VALUE);
    double right_velocity = SaturationCheck(right,SATURATION_VALUE);
    //cout << "left " << left  << " right: " << right << endl;

    //delete this
    //is for testing
    // cout << "printing all claimed" << endl;
    // for(int j = 0; j < claimed_anchor_points.size(); j++)
    // {
    //     cout << "x= "<< claimed_anchor_points.at(j).x << " :: y= " << claimed_anchor_points.at(j).y << endl;
    // } 
    //end testing

    sendDriveCommand(left_velocity, right_velocity);
}

int SaturationCheck(int direction, int sat) {
    if (direction > sat) { direction = sat; }
    if (direction < -sat) { direction = -sat; }
    return direction;
}

float Wrap(float theta) {
    if (theta > M_PI) {
        theta -= 2 * M_PI;
    }
    else if (theta < -M_PI) {
        theta += 2 * M_PI;
    }
    return theta;
}

float GetNewHeading(int side) {

    float max = (M_PI/180) * 135;
    float min = (M_PI/180) * 45;
    float rotation_amount = (min) + (rand()/(RAND_MAX/(max-min)));
    // cout << "rotating " << rotation_amount*(180/M_PI) << endl;
    float theta = 0;
    if (side == RIGHT) // right sonar
    {
        // cout << "RIGHT" << endl;
        theta = Wrap(current_location_odom.theta + rotation_amount);
    }
    else if (side == LEFT)
    {
        // cout << "LEFT" << endl;
        theta = Wrap(current_location_odom.theta - rotation_amount);
    }
    else if (side == CENTER)
    {
        // cout << "CENTER" << endl;
        float max = (M_PI/180) * 225;
        float min = (M_PI/180) * 135;
        float rotation_amount = (min) + (rand()/(RAND_MAX/(max-min)));
        theta = Wrap(current_location_odom.theta + rotation_amount);
    }
    return theta;
}

void openFingers() {
    std_msgs::Float32 angle;
    angle.data = M_PI_2; //open fingers;
    finger_angle_publisher.publish(angle);
}

void closeFingers() {
    std_msgs::Float32 angle;
    angle.data = 0.0;
    finger_angle_publisher.publish(angle);
}

void raiseWrist() {
    std_msgs::Float32 angle;
    angle.data =  0.0; //raise wrist
    wrist_angle_publisher.publish(angle);
}

void lowerWrist() {
    std_msgs::Float32 angle;
    angle.data = 1.24; // down
    wrist_angle_publisher.publish(angle);
}

void centerWrist() {
    std_msgs::Float32 angle;
    angle.data = 0.5; // middle
    wrist_angle_publisher.publish(angle);    
}

Point ClosestAnchorPoint() {
    // Use 4 side anchor points first, and only use the 4 corners when all 4 sides are taken. Which should only happen when there are more than 4 bots running at one time
    Point closest;
    float distance = FLT_MAX;
    geometry_msgs::Pose2D temp;
    for(int i = 0; i < anchor_points.size(); i++) {
        temp.x = anchor_points.at(i).x;
        temp.y = anchor_points.at(i).y;
        temp.theta = 0;
        if (hypot(anchor_points.at(i).y - current_location_odom.y, anchor_points.at(i).x - current_location_odom.x) < distance && ContainsClaimedAnchorPoints(temp) == -1){
            distance = hypot(anchor_points.at(i).y - current_location_odom.y, anchor_points.at(i).x - current_location_odom.x);
            closest = anchor_points.at(i);
        }
    }
    if (claimed_anchor_points.size() == anchor_points.size()) {
        for(int i = 0; i < extra_anchor_points.size(); i++) {
            temp.x = extra_anchor_points.at(i).x;
            temp.y = extra_anchor_points.at(i).y;
            temp.theta = 0;
            if (hypot(extra_anchor_points.at(i).y - current_location_odom.y, extra_anchor_points.at(i).x - current_location_odom.x) < distance && ContainsClaimedAnchorPoints(temp) == -1){
                distance = hypot(extra_anchor_points.at(i).y - current_location_odom.y, extra_anchor_points.at(i).x - current_location_odom.x);
                closest = extra_anchor_points.at(i);
            }
        }
    }
    return closest;
}

bool CheckDriveDistance(float desired_distance) {
    return (hypot(starting_drive_location.x - current_location_odom.x, starting_drive_location.y - current_location_odom.y) > desired_distance); 
}

void SubtractLocationOffset() {
    //might need to adjust anchor points as well?
    resource_covering_home.x -= location_offset_odom.x;
    resource_covering_home.y -= location_offset_odom.y;
    pickup_goal_location.x -= location_offset_odom.x;
    pickup_goal_location.y -= location_offset_odom.y;
    cluster_location.x -= location_offset_odom.x;
    cluster_location.y -= location_offset_odom.y;
    traverse_goal_location.x -= location_offset_odom.x;
    traverse_goal_location.y -= location_offset_odom.y;
    dropoff_goal_location.x -= location_offset_odom.x;
    dropoff_goal_location.y -= location_offset_odom.y;
    dropoff_backup_location.x -= location_offset_odom.x;
    dropoff_backup_location.y -= location_offset_odom.y;
    starting_drive_location.x -= location_offset_odom.x;
    starting_drive_location.y -= location_offset_odom.y;
}

void AddLocationOffset() {
    resource_covering_home.x += location_offset_odom.x;
    resource_covering_home.y += location_offset_odom.y;
    pickup_goal_location.x += location_offset_odom.x;
    pickup_goal_location.y += location_offset_odom.y;
    cluster_location.x += location_offset_odom.x;
    cluster_location.y += location_offset_odom.y;
    traverse_goal_location.x += location_offset_odom.x;
    traverse_goal_location.y += location_offset_odom.y;
    dropoff_goal_location.x += location_offset_odom.x;
    dropoff_goal_location.y += location_offset_odom.y;
    dropoff_backup_location.x += location_offset_odom.x;
    dropoff_backup_location.y += location_offset_odom.y;
    starting_drive_location.x += location_offset_odom.x;
    starting_drive_location.y += location_offset_odom.y;
}

bool PickupBreakout() {
    breakout_counter++;
    if (breakout_counter >= PICK_UP_BREAKOUT_THRESHOLD) {
        
        return true;
    }
    else {
        return false;
    }
}

bool AnchorPointBreakout() {
    breakout_counter++;
    if (breakout_counter >= ANCHOR_POINT_BREAKOUT_THRESHOLD) {
        
        return true;
    }
    else {
        return false;
    }
}

bool DropOffBreakout() {
    breakout_counter++;
    if (breakout_counter >= DROP_OFF_BREAKOUT_THRESHOLD) {
        
        return true;
    }
    else {
        return false;
    }
}

bool ClusterBreakout() {
    breakout_counter++;
    if (breakout_counter >= CLUSTER_BREAKOUT_THRESHOLD) {
        
        return true;
    }
    else {
        return false;
    }
}

void ResetFlags() {
    home_encountered = false;
    resource_encountered = false;
    obstacle_encountered = false;
    //unclaim anchor points if one was claimed
}

int ContainsClaimedAnchorPoints(geometry_msgs::Pose2D anchor_point)
{
    int found = -1;
    for(int i = 0; i < claimed_anchor_points.size(); i++)
    {
        //cout << claimed_anchor_points.at(i).x << " :: " << anchor_point.x << " ||||| " << claimed_anchor_points.at(i).y << " :: " << anchor_point.y << endl; 
        if(fabs(claimed_anchor_points.at(i).x - anchor_point.x) <= .1 && fabs(claimed_anchor_points.at(i).y - anchor_point.y) <= .1)
        {
            found = i;
            cout << "stfu mark point claimed" << endl;
            return found; //anchor point has been claimed
        }
        cout << "anchor point has NOT been claimed" << endl;
    }
    return found;
}

void StateLogger(string state) {
    string file_path = "/home/csadmin/swarmathon_2018/" + published_name + "_state_log_" + std::to_string(timer_start_time) + ".txt"; 
    ofstream world_file; 
 
    world_file.open(file_path, ios::app); 
    if (world_file.is_open()) { 
        world_file << state << " " << std::to_string(GetROSTimeInMilliSecs()) << "\n"; 
        world_file.close(); 
    }  
}