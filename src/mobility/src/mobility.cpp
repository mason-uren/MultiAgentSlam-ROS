#include <ros/ros.h>

// ROS libraries
#include <angles/angles.h>
#include <random_numbers/random_numbers.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

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

// Include Controllers
#include "PickUpController.h"
#include "DropOffController.h"
#include "SearchController.h"
#include "PIDController.h"

// To handle shutdown signals so the node quits
// properly in response to "rosnode kill"
#include <ros/ros.h>
#include <signal.h>
//#include <string>
#include <vector>

using namespace std;

// Random number generator
random_numbers::RandomNumberGenerator* rng;

// Create controllers
PickUpController pickUpController;
DropOffController dropOffController;
SearchController searchController;
PIDController pidController;

// Mobility Logic Functions
void sendDriveCommand(double linearVel, double angularVel);
void openFingers(); // Open fingers to 90 degrees
void closeFingers();// Close fingers to 0 degrees
void raiseWrist();  // Return wrist back to 0 degrees
void lowerWrist();  // Lower wrist to 50 degrees
void mapAverage();  // constantly averages last 100 positions from map

// Numeric Variables for rover positioning
geometry_msgs::Pose2D currentLocation;
geometry_msgs::Pose2D currentLocationMap;
geometry_msgs::Pose2D currentLocationMovingAvg;
geometry_msgs::Pose2D currentLocationMovingAvgOld;
geometry_msgs::Pose2D currentLocationAverage;
geometry_msgs::Pose2D goalLocation;
geometry_msgs::Pose2D alternativeLocation;

geometry_msgs::Pose2D centerLocation;
geometry_msgs::Pose2D centerLocationMap;
geometry_msgs::Pose2D centerLocationOdom;

// For calibrating where home is
struct caliData {
    double posTheta, posX, posY;//angle, x, and y when tags were sighted
    int count;//number of home
    double avgX, avgY;
};
double startingAngle;
vector<caliData> caliVector;
//bool updateCaliVector = false;
double wrapAngle(double angle);

int currentMode = 0;
//int currentHomeTags = 0;
float mobilityLoopTimeStep = 0.1; // time between the mobility loop calls
float status_publish_interval = 1;
float killSwitchTimeout = 10;
double previousDifference = 0;
bool targetDetected = false;
bool targetCollected = false;
float heartbeat_publish_interval = 2;

// Set true when the target block is less than targetDist so we continue
// attempting to pick it up rather than switching to another block in view.
bool lockTarget = false;

// Failsafe state. No legitimate behavior state. If in this state for too long
// return to searching as default behavior.
bool timeOut = false;

// Set to true when the center ultrasound reads less than 0.14m. Usually means
// a picked up cube is in the way.
bool blockBlock = false;

// central collection point has been seen (aka the nest)
bool centerSeen = false;

// Set true when we are insie the center circle and we need to drop the block,
// back out, and reset the boolean cascade.
bool reachedCollectionPoint = false;

// used for calling code once but not in main
bool init = false;

// used to remember place in mapAverage array
int mapCount = 0;

// How many points to use in calculating the map average position
const unsigned int mapHistorySize = 500;
//const unsigned int movingAverageHistorySize = 5;

// An array in which to store map positions
geometry_msgs::Pose2D mapLocation[mapHistorySize];
//geometry_msgs::Pose2D poseHistory[movingAverageHistorySize];

bool avoidingObstacle = false;
bool divertingFromSearchPath = false;

float searchVelocity = 0.2; // meters/second

// New variables for improved obstacle avoidance and pickup of targets while keeping with waypoint based search
bool obstacleEncountered = false;
bool targetEncountered = false;
geometry_msgs::Pose2D divergentLocation;
geometry_msgs::Pose2D targetDivergentLocation;
geometry_msgs::Pose2D obstacleDivergentLocation;
//divergentLocation.x = 1000; // we don't want this to be set to 0 initially because our first goal location after pickup is 0.
//divergentLocation.y = 1000;
std_msgs::String msg;



// state machine states

#define STATE_MACHINE_TRANSFORM 0
#define STATE_MACHINE_ROTATE 1
#define STATE_MACHINE_SKID_STEER 2
#define STATE_MACHINE_PICKUP 3
#define STATE_MACHINE_DROPOFF 4
#define STATE_MACHINE_CALIBRATION 5
#define STATE_MACHINE_INIT 6
#define STATE_MACHINE_GO_TO_WAYPOINT 7
#define STATE_MACHINE_PICKUP_TARGET 8
#define STATE_MACHINE_GO_HOME 9
#define STATE_MACHINE_DROPOFF_TARGET 10
#define STATE_MACHINE_GO_TO_TARGET_DIVERGENT_WAYPOINT 11
#define STATE_MACHINE_GO_TO_OBSTACLE_DIVERGENT_WAYPOINT 12
#define STATE_MACHINE_GO_TO_ALTERNATIVE_WAYPOINT 13

int stateMachineState = STATE_MACHINE_INIT;
int stateMachinePreviousState = STATE_MACHINE_INIT;

geometry_msgs::Twist velocity;
char host[128];
string publishedName;
char prev_state_machine[128];

// Publishers
ros::Publisher stateMachinePublish;
ros::Publisher status_publisher;
ros::Publisher fingerAnglePublish;
ros::Publisher wristAnglePublish;
ros::Publisher infoLogPublisher;
ros::Publisher driveControlPublish;
ros::Publisher heartbeatPublisher;
ros::Publisher goalLocationPublish;
ros::Publisher currentLocationPublish;
ros::Publisher rotationalErrorPublish;
ros::Publisher translationalErrorPublish;
ros::Publisher translationalVelocityPublish;
ros::Publisher rotationalVelocityPublish;
ros::Publisher wrapAngleDifferencePublish;
ros::Publisher leaderElectionPublish;
ros::Publisher pickupCommandPublish;
ros::Publisher stackSizePublish;
ros::Publisher stackDebugPublish;
ros::Publisher obstacleHandlerPublish;

// Subscribers
ros::Subscriber joySubscriber;
ros::Subscriber modeSubscriber;
ros::Subscriber targetSubscriber;
ros::Subscriber obstacleSubscriber;
ros::Subscriber odometrySubscriber;
ros::Subscriber mapSubscriber;
ros::Subscriber leaderElectionSubscriber;

// Timers
ros::Timer stateMachineTimer;
ros::Timer publish_status_timer;
ros::Timer targetDetectedTimer;
ros::Timer publish_heartbeat_timer;

// records time for delays in sequanced actions, 1 second resolution.
time_t timerStartTime;

// An initial delay to allow the rover to gather enough position data to
// average its location.
unsigned int startDelayInSeconds = 1;
float timerTimeElapsed = 0;

time_t timeOfLastObstacle;
//float timeSinceLastObstacle = 0;
int consecutiveObstacleDetections = 0;
const unsigned int OBSTACLE_TIME_THRESHOLD = 1;
const unsigned int CONSECUTIVE_OBSTACLE_THRESHOLD = 5;
const float HOME_RADIUS = 1.0; // Really it is sqrt(0.5) but I want to add a little margin to account for GPS errors.

//Transforms
tf::TransformListener *tfListener;

// OS Signal Handler
void sigintEventHandler(int signal);

//Callback handlers
void joyCmdHandler(const sensor_msgs::Joy::ConstPtr& message);
void modeHandler(const std_msgs::UInt8::ConstPtr& message);
void targetHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& tagInfo);
void obstacleHandler(const std_msgs::UInt8::ConstPtr& message);
void odometryHandler(const nav_msgs::Odometry::ConstPtr& message);
void mapHandler(const nav_msgs::Odometry::ConstPtr& message);
void mobilityStateMachine(const ros::TimerEvent&);
void publishStatusTimerEventHandler(const ros::TimerEvent& event);
void targetDetectedReset(const ros::TimerEvent& event);
void publishHeartBeatTimerEventHandler(const ros::TimerEvent& event);

double getRotationalVelocity();
double getRotationalError();
double getTranslationalVelocity();
double getTranslationalError();
bool compareLocations(geometry_msgs::Pose2D locationOne, geometry_msgs::Pose2D locationTwo, double threshold);
int main(int argc, char **argv) {

    gethostname(host, sizeof (host));
    string hostname(host);

    // instantiate random number generator
    rng = new random_numbers::RandomNumberGenerator();

    centerLocation.x = 0;
    centerLocation.y = 0;
    centerLocationOdom.x = 0;
    centerLocationOdom.y = 0;

    for (int i = 0; i < 100; i++) {
        mapLocation[i].x = 0;
        mapLocation[i].y = 0;
        mapLocation[i].theta = 0;
    }

    if (argc >= 2) {
        publishedName = argv[1];
        cout << "Welcome to the world of tomorrow " << publishedName
             << "!  Mobility turnDirectionule started." << endl;
    } else {
        publishedName = hostname;
        cout << "No Name Selected. Default is: " << publishedName << endl;
    }

    searchController.setStack(publishedName);
    //    //set initial random heading
    //    goalLocation.theta = rng->uniformReal(0, 2 * M_PI); //goalLocation.theta = atan2(goalLocation.y,goalLocation.x);

    //    //select initial search position 50 cm from center (0,0)
    //    goalLocation.x = 0.5 * cos(goalLocation.theta+M_PI); //goalLocation.x = 5.0;
    //    goalLocation.y = 0.5 * sin(goalLocation.theta+M_PI); //goalLocation.y = -5.0;


    //set goal locations based on waypoints for bots
    //goalLocation.x = SearchController::waypoints_x2_final[0];
    //goalLocation.y = SearchController::waypoints_y2_final[0];
    //goalLocation.theta = atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x);
    //goalLocation = searchController.waypointNextLocation(currentLocation, publishedName);
    // NoSignalHandler so we can catch SIGINT ourselves and shutdown the node
    ros::init(argc, argv, (publishedName + "_MOBILITY"), ros::init_options::NoSigintHandler);
    ros::NodeHandle mNH;

    // Register the SIGINT event handler so the node can shutdown properly
    signal(SIGINT, sigintEventHandler);

    joySubscriber = mNH.subscribe((publishedName + "/joystick"), 10, joyCmdHandler);
    modeSubscriber = mNH.subscribe((publishedName + "/mode"), 1, modeHandler);
    targetSubscriber = mNH.subscribe((publishedName + "/targets"), 10, targetHandler);
    obstacleSubscriber = mNH.subscribe((publishedName + "/obstacle"), 10, obstacleHandler);
    odometrySubscriber = mNH.subscribe((publishedName + "/odom/filtered"), 10, odometryHandler);
    mapSubscriber = mNH.subscribe((publishedName + "/odom/ekf"), 10, mapHandler);

    status_publisher = mNH.advertise<std_msgs::String>((publishedName + "/status"), 1, true);
    stateMachinePublish = mNH.advertise<std_msgs::String>((publishedName + "/state_machine"), 1, true);
    fingerAnglePublish = mNH.advertise<std_msgs::Float32>((publishedName + "/fingerAngle/cmd"), 1, true);
    wristAnglePublish = mNH.advertise<std_msgs::Float32>((publishedName + "/wristAngle/cmd"), 1, true);
    infoLogPublisher = mNH.advertise<std_msgs::String>("/infoLog", 1, true);
    driveControlPublish = mNH.advertise<geometry_msgs::Twist>((publishedName + "/driveControl"), 10);
    goalLocationPublish = mNH.advertise<std_msgs::String>((publishedName + "/goalLocation"),1,true);
    currentLocationPublish = mNH.advertise<std_msgs::String>((publishedName + "/currentLocation"),1,true);
    pickupCommandPublish = mNH.advertise<std_msgs::String>((publishedName + "/pickupCommand"),1,true);
    rotationalErrorPublish = mNH.advertise<std_msgs::Float32>((publishedName + "/ourData/rotError"), 1, true);
    rotationalVelocityPublish = mNH.advertise<std_msgs::Float32>((publishedName + "/ourData/rotVel"), 1, true);
    translationalErrorPublish = mNH.advertise<std_msgs::Float32>((publishedName + "/ourData/translateError"), 1, true);
    translationalVelocityPublish = mNH.advertise<std_msgs::Float32>((publishedName + "/ourData/translateVel"), 1, true);
    wrapAngleDifferencePublish = mNH.advertise<std_msgs::Float32>((publishedName + "/ourData/wrapAngle"),1,true);
    heartbeatPublisher = mNH.advertise<std_msgs::String>((publishedName + "/mobility/heartbeat"), 1, true);
    obstacleHandlerPublish = mNH.advertise<std_msgs::String>((publishedName + "/obstacleHandler"), 1, true);
    publish_status_timer = mNH.createTimer(ros::Duration(status_publish_interval), publishStatusTimerEventHandler);
    stateMachineTimer = mNH.createTimer(ros::Duration(mobilityLoopTimeStep), mobilityStateMachine);
    targetDetectedTimer = mNH.createTimer(ros::Duration(0), targetDetectedReset, true);
    stackSizePublish = mNH.advertise<std_msgs::String>((publishedName + "/stackSize"), 1, true);
    stackDebugPublish = mNH.advertise<std_msgs::String>((publishedName + "/stackDebug"), 1, true);
    // Constructors
    //searchController=SearchController::SearchController();
    //pidController=PIDController::PIDController();

    publish_heartbeat_timer = mNH.createTimer(ros::Duration(heartbeat_publish_interval), publishHeartBeatTimerEventHandler);

    tfListener = new tf::TransformListener();
    std_msgs::String msg;
    msg.data = "Log Started";
    infoLogPublisher.publish(msg);

    stringstream ss;
    ss << "Rover start delay set to " << startDelayInSeconds << " seconds";
    msg.data = ss.str();
    infoLogPublisher.publish(msg);

    timerStartTime = time(0);

    ros::spin();

    return EXIT_SUCCESS;
}


// This is the top-most logic control block organised as a state machine.
// This function calls the dropOff, pickUp, and search controllers.
// This block passes the goal location to the proportional-integral-derivative
// controllers in the abridge package.
void mobilityStateMachine(const ros::TimerEvent&) {

    std_msgs::String stackSizeMsg;
    std::ostringstream ss2;
    ss2 << "stack size: " << searchController.getStackSize();
    stackSizeMsg.data = ss2.str();
    stackSizePublish.publish(stackSizeMsg);

    std_msgs::String stateMachineMsg;
    std_msgs::String currentLocationMsg;
    std_msgs::String pickupCommandMsg;
    float rotateOnlyAngleTolerance = 0.4; // was 0.4
    int returnToSearchDelay = 5;
    double waypointDistanceTheshold = 0.1;
    // calls the averaging function, also responsible for
    // transform from Map frame to odom frame.
    mapAverage();
    // Robot is in automode
    if (currentMode == 2 || currentMode == 3) {
        // time since timerStartTime was set to current time
        timerTimeElapsed = time(0) - timerStartTime;
        // init code goes here. (code that runs only once at start of
        // auto mode but wont work in main goes here)
        if (!init) {
            if (timerTimeElapsed > startDelayInSeconds) {
                // Set the location of the center circle location in the map
                // frame based upon our current average location on the map.
                centerLocationMap.x = currentLocationAverage.x;
                centerLocationMap.y = currentLocationAverage.y;
                centerLocationMap.theta = currentLocationAverage.theta;
                startingAngle = currentLocation.theta;
                currentLocationMovingAvgOld.x = currentLocationAverage.x;
                currentLocationMovingAvgOld.y = currentLocationAverage.y;
                previousDifference = wrapAngle( currentLocation.theta - startingAngle);
                goalLocation = searchController.peekWaypoint(currentLocation);
                goalLocation.theta = atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x);
                // initialization has run
                init = true;
            } else {
                return;
            }

        }

        // If no collected or detected blocks set fingers
        // to open wide and raised position.
        if (!targetCollected && !targetDetected) {
            // set gripper
            std_msgs::Float32 angle;

            // open fingers
            angle.data = M_PI_2;

            fingerAnglePublish.publish(angle);
            angle.data = 0;

            // raise wrist
            wristAnglePublish.publish(angle);
        }

        // We want to publish our current location every loop iteration of the mobility state machine.
        std::ostringstream ssCL;
        ssCL<< "currentLocation: "<<currentLocation.x<<", "<<currentLocation.y<<", "<<currentLocation.theta;
        currentLocationMsg.data = ssCL.str();
        currentLocationPublish.publish(currentLocationMsg);

        std_msgs::String goalLocationMsg;
        std::ostringstream ss;
        ss<< "goalLocation: "<<goalLocation.x<<", "<<goalLocation.y<<", "<<goalLocation.theta;
        goalLocationMsg.data = ss.str();
        goalLocationPublish.publish(goalLocationMsg);

        if (time(0)-timeOfLastObstacle>OBSTACLE_TIME_THRESHOLD)
        {
            consecutiveObstacleDetections = 0;
        }
        else if (consecutiveObstacleDetections >= CONSECUTIVE_OBSTACLE_THRESHOLD)
        {
            obstacleEncountered = true;
            avoidingObstacle = true; // not sure what this does, but I think we need it here.
        }

        std_msgs::String obstacleHandlerMsg;
        std::ostringstream ssOH;
        ssOH<< "consecutiveDetections: "<<consecutiveObstacleDetections<<", "<<"timeSinceLastObstacle"<<", "<<time(0)-timeOfLastObstacle<<", "<<"obstacleEncountered, "<<obstacleEncountered;
        obstacleHandlerMsg.data = ssOH.str();
        obstacleHandlerPublish.publish(obstacleHandlerMsg);

        // Select rotation or translation based on required adjustment
        switch(stateMachineState) {
        case STATE_MACHINE_INIT:
        {
            stateMachineMsg.data = "INIT";
            goalLocation = searchController.peekWaypoint(currentLocation);
            goalLocation.theta = atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x);
            pidController.resetTranslationalIntegrator();
            stateMachineState = STATE_MACHINE_GO_TO_WAYPOINT;
            stateMachinePreviousState = STATE_MACHINE_INIT;
            break;
        }
        case STATE_MACHINE_GO_TO_WAYPOINT:
        {
            stateMachineMsg.data = "GO TO WAYPOINT";
            if(obstacleEncountered)
            {
                if (!divertingFromSearchPath)
                {
                    obstacleDivergentLocation = currentLocation;
                    divertingFromSearchPath = true;
                }
                stateMachineState = STATE_MACHINE_GO_TO_ALTERNATIVE_WAYPOINT;
                stateMachinePreviousState = STATE_MACHINE_GO_TO_WAYPOINT;
                break;
            }
            else if(targetEncountered)
            {
                if (!divertingFromSearchPath)
                {
                    targetDivergentLocation = currentLocation;
                    divertingFromSearchPath = true;
                }
                stateMachineState = STATE_MACHINE_PICKUP_TARGET;
                stateMachinePreviousState = STATE_MACHINE_GO_TO_WAYPOINT;
                break;
            }
            else
            {
                // Calculate the diffrence between current and desired heading in radians.
                float errorYaw = getRotationalError();           // If angle > 0.4 radians rotate but dont drive forward.
                float linearVelocity = getTranslationalVelocity();
                if (fabs(errorYaw) > rotateOnlyAngleTolerance) {
                    // rotate but dont drive  0.05 is to prevent turning in reverse
                    sendDriveCommand(0.05, errorYaw/2);
                    stateMachineState = STATE_MACHINE_GO_TO_WAYPOINT;
                    break;
                } else {
                    // move to differential drive step
                    if (!compareLocations(goalLocation,currentLocation,0.2)){
                        // drive and turn simultaniously
                        sendDriveCommand(linearVelocity, errorYaw/2);
                        stateMachineState = STATE_MACHINE_GO_TO_WAYPOINT;
                    }
                    else
                    {
                        // stop
                        sendDriveCommand(0.0, 0.0);
                        searchController.popWaypoint(currentLocation); // We made it to our waypoint, so pop.
                        stateMachineState = STATE_MACHINE_INIT;
                        stateMachinePreviousState = STATE_MACHINE_GO_TO_WAYPOINT;
                    }
                    break;
                }
            }
        }
        case STATE_MACHINE_PICKUP_TARGET:
        {
            stateMachineMsg.data = "PICKUP TARGET";
            if(obstacleEncountered)
            {
                stateMachineState = STATE_MACHINE_GO_TO_ALTERNATIVE_WAYPOINT;
                stateMachinePreviousState = STATE_MACHINE_PICKUP_TARGET;
                pidController.resetTranslationalIntegrator();
                break;
            }
            else
            {
                PickUpResult result;
                // we see a block and have not picked one up yet
                if (targetDetected && !targetCollected) {
                    result = pickUpController.pickUpSelectedTarget(blockBlock);
                    sendDriveCommand(result.cmdVel,result.angleError);
                    std_msgs::Float32 angle;
                    if (result.fingerAngle != -1) {
                        angle.data = result.fingerAngle;
                        fingerAnglePublish.publish(angle);
                    }
                    if (result.wristAngle != -1) {
                        angle.data = result.wristAngle;
                        // raise wrist
                        wristAnglePublish.publish(angle);
                    }
                    if (result.giveUp) {
                        targetDetected = false;
                        stateMachineState = STATE_MACHINE_GO_TO_TARGET_DIVERGENT_WAYPOINT;// I think this is where we give up and we should go back to waypoints again.
                        pidController.resetTranslationalIntegrator();
                        sendDriveCommand(0.0,0);
                        pickUpController.reset();
                    }
                    if (result.pickedUp) {
                        pickUpController.reset();
                        // assume target has been picked up by gripper
                        targetCollected = true;
                        result.pickedUp = false;
                        stateMachineState = STATE_MACHINE_GO_HOME;
                        pidController.resetTranslationalIntegrator();
                        targetEncountered = true;
                        // lower wrist to avoid ultrasound sensors
                        std_msgs::Float32 angle;
                        angle.data = 0.8;
                        wristAnglePublish.publish(angle);
                        sendDriveCommand(0.0,0);
                        return;
                    }                   
                }
                else
                {
                    stateMachineState = stateMachinePreviousState;
                    targetEncountered = false;
                }
                break;
            }
        }
        case STATE_MACHINE_GO_HOME:
        {
            stateMachineMsg.data = "GO HOME";
            if(obstacleEncountered)
            {
                stateMachineState = STATE_MACHINE_GO_TO_ALTERNATIVE_WAYPOINT;
                stateMachinePreviousState = STATE_MACHINE_GO_HOME;
                break;
                pidController.resetTranslationalIntegrator();
            }
            else
            {
                if (!centerSeen)
                {
                    // set center as goal position
//                    goalLocation.x = centerLocationOdom.x;// = 0;
//                    goalLocation.y = centerLocationOdom.y;
//                    goalLocation.theta = atan2(centerLocationOdom.y - currentLocation.y, centerLocationOdom.x - currentLocation.x);

                    goalLocation.x = 0.0;
                    goalLocation.y = 0.0;
                    goalLocation.theta = atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x);

                    // Calculate the diffrence between current and desired heading in radians.
                    float errorYaw = getRotationalError();           // If angle > 0.4 radians rotate but dont drive forward.
                    float linearVelocity = getTranslationalVelocity();
                    if (fabs(errorYaw) > rotateOnlyAngleTolerance) {
                        // rotate but dont drive  0.05 is to prevent turning in reverse
                        sendDriveCommand(0.05, errorYaw);
                        stateMachineState = STATE_MACHINE_GO_HOME;
                        break;
                    } else {
                        // move to differential drive step
                        if (!compareLocations(goalLocation,currentLocation,0.2)){
                            // drive and turn simultaniously
                            sendDriveCommand(linearVelocity, errorYaw/2);
                            stateMachineState = STATE_MACHINE_GO_HOME;
                        }
                        else
                        {
                            // stop
                            sendDriveCommand(0.0, 0.0);
                            stateMachineState = STATE_MACHINE_DROPOFF_TARGET;
                            stateMachinePreviousState = STATE_MACHINE_GO_HOME;
                        }
                        break;
                    }
                }
                else
                {
                    sendDriveCommand(0.0, 0.0);
                    stateMachineState = STATE_MACHINE_DROPOFF_TARGET;
                    stateMachinePreviousState = STATE_MACHINE_GO_HOME;
                    break;
                }
            }
        }
        case STATE_MACHINE_DROPOFF_TARGET:
        {
            stateMachineMsg.data = "DROPOFF TARGET";
            if(obstacleEncountered)
            {
                stateMachineState = STATE_MACHINE_GO_TO_ALTERNATIVE_WAYPOINT;
                stateMachinePreviousState = STATE_MACHINE_DROPOFF_TARGET;
                pidController.resetTranslationalIntegrator();
                break;
            }
            else
            {
                // If returning with a target
                if (targetCollected && !avoidingObstacle) {
                    // calculate the euclidean distance between
                    // centerLocation and currentLocation
                    dropOffController.setCenterDist(hypot(centerLocationOdom.x - currentLocation.x, centerLocationOdom.y - currentLocation.y));
                    dropOffController.setDataLocations(centerLocationOdom, currentLocation, timerTimeElapsed);

                    DropOffResult result = dropOffController.getState();

                    if (result.timer) {
                        timerStartTime = time(0);
                        reachedCollectionPoint = true;
                    }

                    std_msgs::Float32 angle;

                    if (result.fingerAngle != -1) {
                        angle.data = result.fingerAngle;
                        fingerAnglePublish.publish(angle);
                    }

                    if (result.wristAngle != -1) {
                        angle.data = result.wristAngle;
                        wristAnglePublish.publish(angle);
                    }

                    if (result.reset) {
                        // We enter this block when the target is dropped off at the base.
                        timerStartTime = time(0);
                        targetCollected = false;
                        targetDetected = false;
                        lockTarget = false;
                        sendDriveCommand(-0.3,0); // added to get robot to backup before turning around.
                        targetEncountered = false;
                        //                    searchController.popWaypoint(currentLocation);
                        //                    goalLocation = searchController.peekWaypoint(currentLocation);

                        // move back to transform step
                        stateMachineState = STATE_MACHINE_GO_TO_TARGET_DIVERGENT_WAYPOINT;
                        pidController.resetTranslationalIntegrator();
                        reachedCollectionPoint = false;
                        //                    centerLocationOdom = currentLocation; //This is too far from center. Turnning off for now.
                        dropOffController.reset();
                        break;
                    } else if (result.goalDriving && timerTimeElapsed >= 5 ) {
                        goalLocation = result.centerGoal;
                        stateMachineState = STATE_MACHINE_GO_HOME;
                        pidController.resetTranslationalIntegrator();
                        centerSeen = false; // This prevents us from coming back to dropoff immediately after transitioning to GO_HOME.
                        timerStartTime = time(0);
                        break;
                    }
                    // we are in precision/timed driving
                    else {
//                        goalLocation = currentLocation; // This is misleading since we aren't actually driving toward the goalLocation.
                        sendDriveCommand(result.cmdVel,result.angleError);
                        stateMachineState = STATE_MACHINE_DROPOFF_TARGET;
                        break;
                    }
                }
            }
        }
        case STATE_MACHINE_GO_TO_OBSTACLE_DIVERGENT_WAYPOINT:
        {
            stateMachineMsg.data = "OBSTACLE DIVERGENT WAYPOINT";
            if(targetEncountered)
            {
                stateMachineState = STATE_MACHINE_PICKUP_TARGET;
                stateMachinePreviousState = STATE_MACHINE_GO_TO_OBSTACLE_DIVERGENT_WAYPOINT;
                pidController.resetTranslationalIntegrator();
                break;
            }
            else if (obstacleEncountered)
            {
                stateMachineState = STATE_MACHINE_GO_TO_ALTERNATIVE_WAYPOINT;
                stateMachinePreviousState = STATE_MACHINE_GO_TO_OBSTACLE_DIVERGENT_WAYPOINT;
                pidController.resetTranslationalIntegrator();
                break;
            }
            else
            {
                goalLocation = obstacleDivergentLocation;
                // Calculate the diffrence between current and desired heading in radians.
                float errorYaw = getRotationalError();           // If angle > 0.4 radians rotate but dont drive forward.
                float linearVelocity = getTranslationalVelocity();
                if (fabs(errorYaw) > rotateOnlyAngleTolerance) {
                    // rotate but dont drive  0.05 is to prevent turning in reverse
                    sendDriveCommand(0.05, errorYaw/2);
                    stateMachineState = STATE_MACHINE_GO_TO_OBSTACLE_DIVERGENT_WAYPOINT;
                    break;
                } else {
                    // move to differential drive step
                    if (!compareLocations(goalLocation,currentLocation,0.2)){
                        // drive and turn simultaniously
                        sendDriveCommand(linearVelocity, errorYaw/2);
                        stateMachineState = STATE_MACHINE_GO_TO_OBSTACLE_DIVERGENT_WAYPOINT;
                    }
                    else
                    {
                        // stop
                        sendDriveCommand(0.0, 0.0);
                        divertingFromSearchPath = false;
                        stateMachineState = STATE_MACHINE_INIT;
                        stateMachinePreviousState = STATE_MACHINE_GO_TO_OBSTACLE_DIVERGENT_WAYPOINT;
                    }
                    break;
                }
            }
        }
        case STATE_MACHINE_GO_TO_ALTERNATIVE_WAYPOINT:
        {
            stateMachineMsg.data = "AVOIDING OSTACLE";
            goalLocation = alternativeLocation; // Make new goal location a new point away from obstacle. alternativeLocation is set in Obstacle Handler.

            // Calculate the diffrence between current and desired heading in radians.
            float errorYaw = getRotationalError();           // If angle > 0.4 radians rotate but dont drive forward.
            float linearVelocity = getTranslationalVelocity();
            if (fabs(errorYaw) > rotateOnlyAngleTolerance) {
                // rotate but dont drive  0.05 is to prevent turning in reverse
                sendDriveCommand(0.05, errorYaw/2);
                stateMachineState = STATE_MACHINE_GO_TO_ALTERNATIVE_WAYPOINT;
                break;
            } else {
                // move to differential drive step
                if (!compareLocations(goalLocation,currentLocation,0.2)){
                    // drive and turn simultaniously
                    sendDriveCommand(linearVelocity, errorYaw/2);
                    stateMachineState = STATE_MACHINE_GO_TO_ALTERNATIVE_WAYPOINT;
                }
                else
                {
                    // stop
                    sendDriveCommand(0.0, 0.0);
                    avoidingObstacle = false;
                    obstacleEncountered = false;
                    if (stateMachinePreviousState == STATE_MACHINE_GO_TO_WAYPOINT)
                    {
                        stateMachineState = STATE_MACHINE_GO_TO_OBSTACLE_DIVERGENT_WAYPOINT;
                        pidController.resetTranslationalIntegrator();
                    }
                    else if (stateMachinePreviousState == STATE_MACHINE_PICKUP_TARGET)
                    {
                        stateMachineState = STATE_MACHINE_PICKUP_TARGET;
                    }
                    else if (stateMachinePreviousState == STATE_MACHINE_GO_HOME)
                    {
                        stateMachineState = STATE_MACHINE_GO_HOME;
                        pidController.resetTranslationalIntegrator();
                    }
                    else if (stateMachinePreviousState == STATE_MACHINE_DROPOFF_TARGET)
                    {
                        if(targetCollected)
                        {
                            stateMachineState = STATE_MACHINE_GO_HOME; // This is intentional.  If you set this to Dropoff the rover will just drop the target outside the collection zone.
                            centerSeen = false; // Also need this to keep the rover from dropping the target as soon as you enter the GO_HOME state.
                            reachedCollectionPoint = false;
                            pidController.resetTranslationalIntegrator();
                        }
                        else
                        {
                            stateMachineState = STATE_MACHINE_DROPOFF; // if we have already placed the target then DROPOFF state will handle tansitioning to next action.
                        }
                    }
                    else if (stateMachinePreviousState == STATE_MACHINE_GO_TO_TARGET_DIVERGENT_WAYPOINT)
                    {
                        stateMachineState = STATE_MACHINE_GO_TO_TARGET_DIVERGENT_WAYPOINT;
                        pidController.resetTranslationalIntegrator();
                    }
                    else if (stateMachinePreviousState == STATE_MACHINE_GO_TO_OBSTACLE_DIVERGENT_WAYPOINT)
                    {
                        stateMachineState = STATE_MACHINE_GO_TO_OBSTACLE_DIVERGENT_WAYPOINT;
                        pidController.resetTranslationalIntegrator();
                    }
                    else
                    {
                        stateMachineState = 911;
                    }
                    stateMachinePreviousState = STATE_MACHINE_GO_TO_ALTERNATIVE_WAYPOINT;
                }
                break;
            }
        }
        case STATE_MACHINE_GO_TO_TARGET_DIVERGENT_WAYPOINT:
        {
            stateMachineMsg.data = "TARGET DIVERGENT WAYPOINT";
            if(obstacleEncountered)
            {
                stateMachineState = STATE_MACHINE_GO_TO_ALTERNATIVE_WAYPOINT;
                stateMachinePreviousState = STATE_MACHINE_GO_TO_TARGET_DIVERGENT_WAYPOINT;
                pidController.resetTranslationalIntegrator();
                break;
            }
            else if(targetEncountered)
            {
                stateMachineState = STATE_MACHINE_PICKUP_TARGET;
                stateMachinePreviousState = STATE_MACHINE_GO_TO_TARGET_DIVERGENT_WAYPOINT;
                pidController.resetTranslationalIntegrator();
                break;
            }
            else
            {
                if(sqrt(currentLocation.x*currentLocation.x + currentLocation.y*currentLocation.y)<=HOME_RADIUS) // We are too close to the home circle.
                {
                    double newTheta = atan2(currentLocation.y, currentLocation.x); // Find the nearest direction out of the circle.
                    goalLocation.x = 1.5*HOME_RADIUS*cos(newTheta); // We just want to get out of the home circle so get out of there.
                    goalLocation.y = 1.5*HOME_RADIUS*sin(newTheta);
                }
                else
                {
                    bool invalidWaypoint = true;
                    int retryCounter = 0;
                    while(invalidWaypoint)
                    {
                        float tDL = atan2(targetDivergentLocation.y - currentLocation.y, targetDivergentLocation.x - currentLocation.x);
                        float distToWaypoint = hypot(targetDivergentLocation.x - currentLocation.x, targetDivergentLocation.y - currentLocation.y);
                        goalLocation.x = currentLocation.x + distToWaypoint*cos(tDL+retryCounter*0.0872665);
                        goalLocation.y = currentLocation.y + distToWaypoint*sin(tDL+retryCounter*0.0872665); // 5 degrees is 0.0872665 radians.
                        goalLocation.theta = atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x);

                        if(!searchController.waypointIntersectsHome(currentLocation, goalLocation))
                        {
                            invalidWaypoint = false;
                        }
                        retryCounter++;
                    }
                }






                // Calculate the diffrence between current and desired heading in radians.
                float errorYaw = getRotationalError();           // If angle > 0.4 radians rotate but dont drive forward.
                float linearVelocity = getTranslationalVelocity();
                if (fabs(errorYaw) > rotateOnlyAngleTolerance) {
                    // rotate but dont drive  0.05 is to prevent turning in reverse
                    sendDriveCommand(0.05, errorYaw/2);
                    stateMachineState = STATE_MACHINE_GO_TO_TARGET_DIVERGENT_WAYPOINT;
                    break;
                } else {
                    // move to differential drive step
                    if (!compareLocations(goalLocation,currentLocation,0.2)){
                        // drive and turn simultaniously
                        sendDriveCommand(linearVelocity, errorYaw/2);
                        stateMachineState = STATE_MACHINE_GO_TO_TARGET_DIVERGENT_WAYPOINT;
                    }
                    else
                    {
                        // stop
                        sendDriveCommand(0.0, 0.0);
                        divertingFromSearchPath = false;
                        stateMachineState = STATE_MACHINE_INIT;
                        stateMachinePreviousState = STATE_MACHINE_GO_TO_TARGET_DIVERGENT_WAYPOINT;
                    }
                    break;
                }
            }
        }
        default:
        {
            stateMachineMsg.data = "DEFAULT HIT";
            break;
        }
        } /* end of switch() */
    }
    // mode is NOT auto
    else {
        // publish current state for the operator to see
        stateMachineMsg.data = "WAITING";
    }

    // publish state machine string for user, only if it has changed, though
//    if (strcmp(stateMachineMsg.data.c_str(), prev_state_machine) != 0) {
        stateMachinePublish.publish(stateMachineMsg);
//        sprintf(prev_state_machine, "%s", stateMachineMsg.data.c_str());
//    }
}

double wrapAngle( double angle )
{
    double twoPi = 2.0 * M_PI;
    return angle - twoPi * floor( angle / twoPi );
}

double getRotationalError()
{
    double rotationalError = angles::shortest_angular_distance(currentLocation.theta, atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x)); // no abs on this error.
//    std_msgs::Float32 rErrorMsg;
//    rErrorMsg.data = rotationalError;
//    rotationalErrorPublish.publish(rErrorMsg);
    return rotationalError;
}
double getTranslationalError()
{
    double translationalError = hypot(goalLocation.x - currentLocation.x, goalLocation.y - currentLocation.y);
//    std_msgs::Float32 tErrorMsg;
//    tErrorMsg.data = translationalError;
//    translationalErrorPublish.publish(tErrorMsg);
    return translationalError;
}

bool compareLocations(geometry_msgs::Pose2D locationOne, geometry_msgs::Pose2D locationTwo, double threshold)
{
    return (abs(locationOne.x-locationTwo.x)<threshold)&&(abs(locationOne.y-locationTwo.y)<threshold);
}

double getTranslationalVelocity()
{
    double translationalVelocity = pidController.calculateTranslationalVelocity(currentLocation,goalLocation);
//    std_msgs::Float32 tVelocityMsg;
//    tVelocityMsg.data = translationalVelocity;
//    translationalVelocityPublish.publish(tVelocityMsg);
    return translationalVelocity;
}

void sendDriveCommand(double linearVel, double angularError)
{
    velocity.linear.x = linearVel,
    velocity.angular.z = angularError;

    // publish the drive commands
    driveControlPublish.publish(velocity);
}

/*************************
 * ROS CALLBACK HANDLERS *
 *************************/

void targetHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message) {

    // If in manual mode do not try to automatically pick up the target
    if (currentMode == 1 || currentMode == 0) return;
    if (stateMachineState == STATE_MACHINE_CALIBRATION)
    {
        double sumX = 0.0, sumY = 0.0;
        int count = 0;
        for (int i = 0; i < message->detections.size(); i++)
        {
            if (message->detections[i].id == 256) {
                // checks if tag is on the right or left side of the image
                count++;
                sumX += message->detections[i].pose.pose.position.x;
                sumY += message->detections[i].pose.pose.position.y;
            }
        }
        if(count > 0)
        {
            caliData newData;
            newData.posTheta = currentLocation.theta;
            newData.posX = currentLocation.x;
            newData.posY = currentLocation.y;
            newData.count = count;
            newData.avgX = sumX/count;
            newData.avgY = sumY/count;
            caliVector.insert(caliVector.begin(),newData);
        }
        //currentHomeTags = count;
        //updateTagVector = true;
    }
    // if a target is detected and we are looking for center tags
    if (message->detections.size() > 0 && !reachedCollectionPoint) {
        float cameraOffsetCorrection = 0.020; //meters;

        centerSeen = false;
        double count = 0;
        double countRight = 0;
        double countLeft = 0;

        // this loop is to get the number of center tags
        for (int i = 0; i < message->detections.size(); i++) {
            if (message->detections[i].id == 256) {
                geometry_msgs::PoseStamped cenPose = message->detections[i].pose;

                // checks if tag is on the right or left side of the image
                if (cenPose.pose.position.x + cameraOffsetCorrection > 0) {
                    countRight++;

                } else {
                    countLeft++;
                }

                centerSeen = true;
                count++;
            }
        }

        if (centerSeen && targetCollected) {
//            stateMachineState = 911; //STATE_MACHINE_PICKUP_TARGET; // This used to be STATE_MACHINE_TRANSFORM
//            goalLocation = currentLocation;
        }

        dropOffController.setDataTargets(count,countLeft,countRight);
        // if we see the center and we dont have a target collected
        if (centerSeen && !targetCollected)
        {
            float centeringTurn = 0.15; //radians
            targetDetected = false;
            pickUpController.reset();
            return;
        }
    }
    // end found target and looking for center tags

    // found a target april tag and looking for april cubes;
    // with safety timer at greater than 5 seconds.
    PickUpResult result;

    if (message->detections.size() > 0 && !targetCollected && timerTimeElapsed > 5) {
        targetDetected = true;
        //add position to stack
        //searchController.waypointSearchFound(currentLocation, goalLocation, publishedName);

        // pickup state so target handler can take over driving.
//        stateMachineState = STATE_MACHINE_PICKUP_TARGET;
        targetEncountered = true;
        result = pickUpController.selectTarget(message);

        std_msgs::Float32 angle;

        if (result.fingerAngle != -1) {
            angle.data = result.fingerAngle;
            fingerAnglePublish.publish(angle);
        }

        if (result.wristAngle != -1) {
            angle.data = result.wristAngle;
            wristAnglePublish.publish(angle);
        }
    }
}

void modeHandler(const std_msgs::UInt8::ConstPtr& message) {
    currentMode = message->data;
    sendDriveCommand(0.0, 0.0);
}

void obstacleHandler(const std_msgs::UInt8::ConstPtr& message) {



    const double PI_OVER_4 = 0.78539816339;

    if ((!targetDetected || targetCollected) && (message->data > 0)) {
        consecutiveObstacleDetections++;
    //    timeSinceLastObstacle = 0;
        timeOfLastObstacle = time(0);

//        std_msgs::String obstacleHandlerMsg;
//        std::ostringstream ssOH;
//        ssOH<< "consecutiveDetections: "<<consecutiveObstacleDetections<<", "<<"timeOfLastObstacle"<<", "<<timeOfLastObstacle;
//        obstacleHandlerMsg.data = ssOH.str();
//        obstacleHandlerPublish.publish(obstacleHandlerMsg);
        // obstacle on right side

        if(sqrt(currentLocation.x*currentLocation.x + currentLocation.y*currentLocation.y)<=HOME_RADIUS) // We are too close to the home circle.
        {
            double newTheta = atan2(currentLocation.y, currentLocation.x); // Find the nearest direction out of the circle.
            alternativeLocation.x = 1.5*HOME_RADIUS*cos(newTheta); // We just want to get out of the home circle so get out of there.
            alternativeLocation.y = 1.5*HOME_RADIUS*sin(newTheta);
        }
        else
        {
            bool invalidWaypoint = true;
            int retryCounter = 0;
            while(invalidWaypoint)
            {
                double distanceToMove = 0.5 + rng->uniformReal(0, 1);
                if (message->data == 1) {
                    // select new heading 0.2 radians to the left
                    //goalLocation.theta = currentLocation.theta + 0.6;
                    alternativeLocation.x = currentLocation.x + distanceToMove*cos(currentLocation.theta + PI_OVER_4+retryCounter*5.0);//hardcoded angle of pi/2 & length of 2, tunable
                    alternativeLocation.y = currentLocation.y + distanceToMove*sin(currentLocation.theta + PI_OVER_4+retryCounter*5.0);//hardcoded angle of pi/2 & length of 2, tunable
                    alternativeLocation.theta = currentLocation.theta + PI_OVER_4+retryCounter*5.0;
                }
                // obstacle in front or on left side
                else if (message->data == 2) {
                    // select new heading 0.2 radians to the right
                    //goalLocation.theta = currentLocation.theta + 0.6;
                    alternativeLocation.x = currentLocation.x + distanceToMove*cos(currentLocation.theta - PI_OVER_4-retryCounter*5.0);
                    alternativeLocation.y = currentLocation.y + distanceToMove*sin(currentLocation.theta - PI_OVER_4-retryCounter*5.0);
                    alternativeLocation.theta = currentLocation.theta - PI_OVER_4-retryCounter*5.0;
                }
                if(!searchController.waypointIntersectsHome(currentLocation, alternativeLocation))
                {
                    invalidWaypoint = false;
                }
                retryCounter++;
            }
        }
    }

    // the front ultrasond is blocked very closely. 0.14m currently
    if (message->data == 4) {
        blockBlock = true;
    } else {
        blockBlock = false;
    }
}

void odometryHandler(const nav_msgs::Odometry::ConstPtr& message) {
    //Get theta rotation by converting quaternion orientation to pitch/roll/yaw
    tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
}

void mapHandler(const nav_msgs::Odometry::ConstPtr& message) {
    //Get (x,y) location directly from pose
    currentLocationMap.x = message->pose.pose.position.x;
    currentLocationMap.y = message->pose.pose.position.y;

    float alpha = 0.5;
    currentLocationMovingAvg.x = alpha * currentLocationMap.x + (1-alpha) * currentLocationMovingAvgOld.x;
    currentLocationMovingAvg.y = alpha * currentLocationMap.y + (1-alpha) * currentLocationMovingAvgOld.y;
    currentLocationMovingAvgOld.x = currentLocationMovingAvg.x;
    currentLocationMovingAvgOld.y = currentLocationMovingAvg.y;
    // The code below is for test only.
    //Get (x,y) location directly from pose
    currentLocation.x = currentLocationMovingAvg.x; //message->pose.pose.position.x;
    currentLocation.y = currentLocationMovingAvg.y; //message->pose.pose.position.y;

    if(publishedName == "ajax")
    {
        currentLocation.x = currentLocationMovingAvg.x + 1.25;
        currentLocation.y = currentLocationMovingAvg.y + 0;
    }
    else if(publishedName == "achilles")
    {
        currentLocation.x = currentLocationMovingAvg.x - 1.25;
        currentLocation.y = currentLocationMovingAvg.y + 0;
    }
    else if(publishedName == "aeneas")
    {
        currentLocation.x = currentLocationMovingAvg.x + 0;
        currentLocation.y = currentLocationMovingAvg.y - 1.25;
    }
    // The code above is for test only.


    //Get theta rotation by converting quaternion orientation to pitch/roll/yaw
    tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    currentLocationMap.theta = yaw;
    currentLocation.theta = yaw;

//    std_msgs::String currentLocationMsg;
//    std::ostringstream ssCL;
//    ssCL<< "currentLocation: "<<currentLocation.x<<", "<<currentLocation.y<<", "<<currentLocation.theta;
//    currentLocationMsg.data = ssCL.str();
//    currentLocationPublish.publish(currentLocationMsg);
}

void joyCmdHandler(const sensor_msgs::Joy::ConstPtr& message) {
    if (currentMode == 0 || currentMode == 1) {
        sendDriveCommand(abs(message->axes[4]) >= 0.1 ? message->axes[4] : 0, abs(message->axes[3]) >= 0.1 ? message->axes[3] : 0);
    }
}


void publishStatusTimerEventHandler(const ros::TimerEvent&) {
    std_msgs::String msg;
    msg.data = "online";
    status_publisher.publish(msg);
}


void targetDetectedReset(const ros::TimerEvent& event) {
    targetDetected = false;

    std_msgs::Float32 angle;
    angle.data = 0;

    // close fingers
    fingerAnglePublish.publish(angle);

    // raise wrist
    wristAnglePublish.publish(angle);
}

void sigintEventHandler(int sig) {
    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}

void mapAverage() {
    // store currentLocation in the averaging array
    mapLocation[mapCount] = currentLocationMap;
    mapCount++;

    if (mapCount >= mapHistorySize) {
        mapCount = 0;
    }

    double x = 0;
    double y = 0;
    double theta = 0;

    // add up all the positions in the array
    for (int i = 0; i < mapHistorySize; i++) {
        x += mapLocation[i].x;
        y += mapLocation[i].y;
        theta += mapLocation[i].theta;
    }

    // find the average
    x = x/mapHistorySize;
    y = y/mapHistorySize;

    // Get theta rotation by converting quaternion orientation to pitch/roll/yaw
    //theta = theta/100; // You can't average angles.  Example Avg(1,359)=180.
    theta = mapLocation[mapHistorySize-1].theta; // I decided to take the most recent heading instead.
    currentLocationAverage.x = x;
    currentLocationAverage.y = y;
    currentLocationAverage.theta = theta;


    // only run below code if a centerLocation has been set by initilization
    if (init) {
        // map frame
        geometry_msgs::PoseStamped mapPose;

        // setup msg to represent the center location in map frame
        mapPose.header.stamp = ros::Time::now();

        mapPose.header.frame_id = publishedName + "/map";
        mapPose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, centerLocationMap.theta);
        mapPose.pose.position.x = centerLocationMap.x;
        mapPose.pose.position.y = centerLocationMap.y;
        geometry_msgs::PoseStamped odomPose;
        string x = "";

        try { //attempt to get the transform of the center point in map frame to odom frame.
            tfListener->waitForTransform(publishedName + "/map", publishedName + "/odom", ros::Time::now(), ros::Duration(1.0));
            tfListener->transformPose(publishedName + "/odom", mapPose, odomPose);
        }

        catch(tf::TransformException& ex) {
            ROS_INFO("Received an exception trying to transform a point from \"map\" to \"odom\": %s", ex.what());
            x = "Exception thrown " + (string)ex.what();
            std_msgs::String msg;
            stringstream ss;
            ss << "Exception in mapAverage() " + (string)ex.what();
            msg.data = ss.str();
            infoLogPublisher.publish(msg);
        }

        // Use the position and orientation provided by the ros transform.
        centerLocation.x = odomPose.pose.position.x; //set centerLocation in odom frame
        centerLocation.y = odomPose.pose.position.y;


    }
}

void publishHeartBeatTimerEventHandler(const ros::TimerEvent&) {
    std_msgs::String msg;
    msg.data = "";
    heartbeatPublisher.publish(msg);
}
