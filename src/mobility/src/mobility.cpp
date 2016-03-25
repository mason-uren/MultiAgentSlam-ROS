#include <ros/ros.h>

//ROS libraries
#include <angles/angles.h>
#include <random_numbers/random_numbers.h>
#include <tf/transform_datatypes.h>

//ROS messages
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

//Custom messages
#include <shared_messages/TagsImage.h>

// To handle shutdown signals so the node quits properly in response to "rosnode kill"
#include <ros/ros.h>
#include <signal.h>

//Custom message passing
#include "messages.hpp"

using namespace std;

//Random number generator
random_numbers::RandomNumberGenerator* rng; 

//Mobility Logic Functions
void setVelocity(double linearVel, double angularVel);

// robot 1
int SERACH_1 = 0;
vector<geometry_msgs::Pose2D> targetDetectedPos_01;

// robot 2
int SERACH_2 = 1;
vector<geometry_msgs::Pose2D> targetDetectedPos_02;

// robot 3 is collecting
int COLLECTOR = 2;

//Numeric Variables
geometry_msgs::Pose2D currentLocation;
geometry_msgs::Pose2D goalLocation;
vector<geometry_msgs::Pose2D> savedPositions;
int currentMode = 0;
float mobilityLoopTimeStep = 0.1; //time between the mobility loop calls
float status_publish_interval = 5;
float killSwitchTimeout = 10;
std_msgs::Int16 targetDetected; //ID of the detected target
std_msgs::Int16 targetCollected; //ID of the collected target
bool targetsCollected [256] = {0}; //array of booleans indicating whether each target ID has been found
bool targetsDetected [256] = {0};
geometry_msgs::Pose2D targetPositions[256];
Path paths[6];

int next = 0;
int ARENA_SIZE = 12;
float waypoints_theta_01 [] = {0.0, 0.0, M_PI_2, M_PI, M_PI_2, 0.0, M_PI_2, M_PI, M_PI_2, 0.0, M_PI_2, M_PI, M_PI_2};
float waypoints_theta_02 [] = {0.0, M_PI, 3.0*M_PI_2, 0.0, 3.0*M_PI_2, M_PI, 3.0*M_PI_2, 0.0, 3.0*M_PI_2, M_PI, 3.0*M_PI_2, 0.0, 3.0*M_PI_2};
float waypoints_x [] = {0.0, 5.5, 5.5, -5.5, -5.5, 5.5, 5.5, -5.5, -5.5, 5.5, 5.5, -5.5, -5.5};
float waypoints_y [] = {0.0, 0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5};

float waypoints_x_02 [] = {0.0, -5.5, -5.5, 5.5, 5.5, -5.5, -5.5, 5.5, 5.5, -5.5, -5.5, 5.5, 5.5};
float waypoints_y_02 [] = {0.0, 0.0, -0.5, -1.0, -1.5, -2.0, -2.5, -3.0, -3.5, -4.0, -4.5, -5.0, -5.5};

// state machine states
#define STATE_MACHINE_TRANSFORM 0
#define STATE_MACHINE_ROTATE    1
#define STATE_MACHINE_TRANSLATE 2
int stateMachineState = STATE_MACHINE_TRANSFORM;

geometry_msgs::Twist velocity;
char host[128];
string publishedName;
string memberNames[6];
int self_idx = -1;
bool sent_name = false;
bool avoiding_obstacle = false;
int swarmSize = 0;
char prev_state_machine[128];

//Publishers
ros::Publisher velocityPublish;
ros::Publisher stateMachinePublish;
ros::Publisher status_publisher;
ros::Publisher targetCollectedPublish;
ros::Publisher messagePublish;
ros::Publisher targetPickUpPublish;
ros::Publisher targetDropOffPublish;

//Subscribers
ros::Subscriber joySubscriber;
ros::Subscriber modeSubscriber;
ros::Subscriber targetSubscriber;
ros::Subscriber obstacleSubscriber;
ros::Subscriber odometrySubscriber;
ros::Subscriber targetsCollectedSubscriber;
ros::Subscriber messageSubscriber;

//Timers
ros::Timer stateMachineTimer;
ros::Timer publish_status_timer;
ros::Timer killSwitchTimer;

// OS Signal Handler
void sigintEventHandler(int signal);

//Callback handlers
void joyCmdHandler(const geometry_msgs::Twist::ConstPtr& message);
void modeHandler(const std_msgs::UInt8::ConstPtr& message);
void targetHandler(const shared_messages::TagsImage::ConstPtr& tagInfo);
void obstacleHandler(const std_msgs::UInt8::ConstPtr& message);
void odometryHandler(const nav_msgs::Odometry::ConstPtr& message);
void mobilityStateMachine(const ros::TimerEvent&);
void publishStatusTimerEventHandler(const ros::TimerEvent& event);
void targetsCollectedHandler(const std_msgs::Int16::ConstPtr& message);
void killSwitchTimerEventHandler(const ros::TimerEvent& event);

void messagePasser(const std_msgs::String::ConstPtr& message);

int main(int argc, char **argv) {

    gethostname(host, sizeof (host));
    string hostname(host);

    rng = new random_numbers::RandomNumberGenerator(); //instantiate random number generator
    
    targetDetected.data = -1; //initialize target detected 

    if (argc >= 2) {
        publishedName = argv[1];
        cout << "Welcome to the world of tomorrow " << publishedName << "!  Mobility module started." << endl;
    } else {
        publishedName = hostname;
        cout << "No Name Selected. Default is: " << publishedName << endl;
    }

    // NoSignalHandler so we can catch SIGINT ourselves and shutdown the node
    ros::init(argc, argv, (publishedName + "_MOBILITY"), ros::init_options::NoSigintHandler);
    ros::NodeHandle mNH;

    signal(SIGINT, sigintEventHandler); // Register the SIGINT event handler so the node can shutdown properly

    joySubscriber = mNH.subscribe((publishedName + "/joystick"), 10, joyCmdHandler);
    modeSubscriber = mNH.subscribe((publishedName + "/mode"), 1, modeHandler);
    targetSubscriber = mNH.subscribe((publishedName + "/targets"), 10, targetHandler);
    obstacleSubscriber = mNH.subscribe((publishedName + "/obstacle"), 10, obstacleHandler);
    odometrySubscriber = mNH.subscribe((publishedName + "/odom/ekf"), 10, odometryHandler);
    targetsCollectedSubscriber = mNH.subscribe(("targetsCollected"), 10, targetsCollectedHandler);
    messageSubscriber = mNH.subscribe(("messages"), 10, messagePasser);

    status_publisher = mNH.advertise<std_msgs::String>((publishedName + "/status"), 1, true);
    velocityPublish = mNH.advertise<geometry_msgs::Twist>((publishedName + "/velocity"), 10);
    stateMachinePublish = mNH.advertise<std_msgs::String>((publishedName + "/state_machine"), 1, true);
    targetCollectedPublish = mNH.advertise<std_msgs::Int16>(("targetsCollected"), 1, true);
    messagePublish = mNH.advertise<std_msgs::String>(("messages"), 10, true);
    targetPickUpPublish = mNH.advertise<sensor_msgs::Image>((publishedName + "/targetPickUpImage"), 1, true);
    targetDropOffPublish = mNH.advertise<sensor_msgs::Image>((publishedName + "/targetDropOffImage"), 1, true);

    publish_status_timer = mNH.createTimer(ros::Duration(status_publish_interval), publishStatusTimerEventHandler);
    killSwitchTimer = mNH.createTimer(ros::Duration(killSwitchTimeout), killSwitchTimerEventHandler);
    stateMachineTimer = mNH.createTimer(ros::Duration(mobilityLoopTimeStep), mobilityStateMachine);
    
    ros::spin();
    
    return EXIT_SUCCESS;
}

void mobilityStateMachine(const ros::TimerEvent&)
{
    std_msgs::String stateMachineMsg;    

    if ((currentMode == 2 || currentMode == 3)) { //Robot is in automode

        switch(stateMachineState) {

            //Select rotation or translation based on required adjustment
            //If no adjustment needed, select new goal
            case STATE_MACHINE_TRANSFORM: {
                stateMachineMsg.data = "TRANSFORMING";
                //If angle between current and goal is significant
                if (fabs(angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta)) > 0.1) {
                    stateMachineState = STATE_MACHINE_ROTATE; //rotate
                }
                //If goal has not yet been reached
                else if (fabs(angles::shortest_angular_distance(currentLocation.theta, atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x))) < M_PI_2) {
                    stateMachineState = STATE_MACHINE_TRANSLATE; //translate
                }
                //If returning with a target -- only robot 3 should be returning with target (collecting) -- anything other than -1 means collect
                else if (targetCollected.data != -1 && self_idx == COLLECTOR) {
                    //If goal has not yet been reached
                    if (hypot(0.0 - currentLocation.x, 0.0 - currentLocation.y) > 0.5) {
                        //set angle to center as goal heading
                        goalLocation.theta = M_PI + atan2(currentLocation.y, currentLocation.x);

                        //set center as goal position
                        goalLocation.x = 0.0;
                        goalLocation.y = 0.0;
                    }
                    //Otherwise, reset target and select new target to collect 
                    else if (self_idx == COLLECTOR){
                        targetCollectedPublish.publish(targetCollected);
                        targetCollected.data = -1; // signal target has been collected
                        targetDetected.data = -1; // signal target has been detected
                    }
                }
                //Otherwise, assign a new goal
                else {
                    if(avoiding_obstacle) { // this is part of assigning new goal, but it is taking into consideration, obstacles
                        avoiding_obstacle = false;

                        while(savedPositions.size() > 1) {
                            savedPositions.pop_back();
                        }

                        goalLocation.x = savedPositions.back().x;
                        goalLocation.y = savedPositions.back().y;
                        goalLocation.theta = savedPositions.back().theta;

                        savedPositions.pop_back();

                    } else {

                        if(swarmSize >= 3 && self_idx == SERACH_1) { // robot 1

                            if(paths[self_idx].Size() == 0) {

                                for(int i = 0; i < ARENA_SIZE; i++) {
                                    paths[self_idx].Add(currentLocation.x, currentLocation.y, currentLocation.theta, waypoints_x[i], waypoints_y[i]);
                                }

                            } else {
                                PathNode* n = paths[self_idx].Get(0);
                                if(n != NULL) {
                                    goalLocation.x = n->Goal().x;
                                    goalLocation.y = n->Goal().y;
                                    goalLocation.theta = n->Goal().theta;
                                }
                                paths[self_idx].Remove(0);
                            }

                        }

                        if(swarmSize >= 3 && self_idx == SERACH_2) { // robot 2

                            if(paths[self_idx].Size() == 0) {

                                for(int i = 0; i < ARENA_SIZE; i++) {
                                    paths[self_idx].Add(currentLocation.x, currentLocation.y, currentLocation.theta, waypoints_x_02[i], waypoints_y_02[i]);
                                }

                            } else {
                                PathNode* n = paths[self_idx].Get(0);
                                if(n != NULL) {
                                    goalLocation.x = n->Goal().x;
                                    goalLocation.y = n->Goal().y;
                                    goalLocation.theta = n->Goal().theta;
                                }
                                paths[self_idx].Remove(0);
                            }
                        
                        }

                        if(swarmSize >= 3 && self_idx == COLLECTOR) { // robot 3 // ajax // white

                            if(paths[self_idx].Size() == 0) {

                                if(targetDetectedPos_01.size() > 0){

                                    for(int i = 0; i < targetDetectedPos_01.size(); i++){

                                        paths[self_idx].Add(currentLocation.x, currentLocation.y, targetDetectedPos_01.back().theta, targetDetectedPos_01.back().x, targetDetectedPos_01.back().y);
                                        targetDetectedPos_01.pop_back();
                                    }

                                }

                            } else {
                                PathNode* n = paths[self_idx].Get(0);
                                if(n != NULL) {
                                    goalLocation.x = n->Goal().x;
                                    goalLocation.y = n->Goal().y;
                                    goalLocation.theta = n->Goal().theta;
                                }
                                paths[self_idx].Remove(0);
                            }
                        
                        }

                    }

                }

                //Purposefully fall through to next case without breaking
            }

            //Calculate angle between currentLocation.theta and goalLocation.theta
            //Rotate left or right depending on sign of angle
            //Stay in this state until angle is minimized
            case STATE_MACHINE_ROTATE: {
                stateMachineMsg.data = "ROTATING";
                if (angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta) > 0.1) {
                    setVelocity(0.0, 0.2); //rotate left
                }
                else if (angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta) < -0.1) {
                    setVelocity(0.0, -0.2); //rotate right
                }
                else {
                    setVelocity(0.0, 0.0); //stop
                    stateMachineState = STATE_MACHINE_TRANSLATE; //move to translate step
                }
                break;
            }

            //Calculate angle between currentLocation.x/y and goalLocation.x/y
            //Drive forward
            //Stay in this state until angle is at least PI/2
            case STATE_MACHINE_TRANSLATE: {
                stateMachineMsg.data = "TRANSLATING";
                if (fabs(angles::shortest_angular_distance(currentLocation.theta, atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x))) < M_PI_2) {
                    setVelocity(0.3, 0.0);
                }
                else {
                    setVelocity(0.0, 0.0); //stop
                    stateMachineState = STATE_MACHINE_TRANSFORM; //move back to transform step
                }
                break;
            }

            default: {
                break;
            }
        }

    } else { // mode is NOT auto

        // publish current state for the operator to see
        stateMachineMsg.data = "WAITING";
    }

    // publish state machine string for user, only if it has changed, though
    if (strcmp(stateMachineMsg.data.c_str(), prev_state_machine) != 0) {
        stateMachinePublish.publish(stateMachineMsg);
        sprintf(prev_state_machine, "%s", stateMachineMsg.data.c_str());
    }

    next++;
}

void setVelocity(double linearVel, double angularVel) 
{
  // Stopping and starting the timer causes it to start counting from 0 again.
  // As long as this is called before the kill swith timer reaches killSwitchTimeout seconds
  // the rover's kill switch wont be called.
  //killSwitchTimer.stop();
  //killSwitchTimer.start();
  
  velocity.linear.x = linearVel * 1.5;
  velocity.angular.z = angularVel * 8; //scaling factor for sim; removed by aBridge node
  velocityPublish.publish(velocity);
}

/***********************
 * ROS CALLBACK HANDLERS
 ************************/

void targetHandler(const shared_messages::TagsImage::ConstPtr& message) {

	//if this is the goal target
	if (message->tags.data[0] == 256) {
        //if we were returning with a target
	    if (targetDetected.data != -1) {
			//publish to scoring code // only robot 3 should be collecting
			targetDropOffPublish.publish(message->image);
			targetDetected.data = -1;
	    }
	}

	//if target has not previously been detected 
	else if (targetDetected.data == -1 && self_idx == SERACH_1) { // robot 1 // detect target, but do not publish, just save position and add to vector 
        
        //check if target has not yet been collected
        if (!targetsCollected[message->tags.data[0]]) {
			//copy target ID to class variable
			targetDetected.data = message->tags.data[0];

            geometry_msgs::Pose2D targetDetectedPos;

            float x = currentLocation.x;
            float y = currentLocation.y;

            targetDetectedPos.x = x;
            targetDetectedPos.y = y;
            targetDetectedPos.theta = currentLocation.theta;

            targetDetectedPos_01.push_back(targetDetectedPos);

            //publish detected target
            targetCollectedPublish.publish(targetDetected);

            //publish to scoring code
            targetPickUpPublish.publish(message->image);
    }

    //if target has not previously been detected 
    else if (targetDetected.data == -1 && self_idx == SERACH_2) { // robot 2
        
        //check if target has not yet been collected
        if (!targetsCollected[message->tags.data[0]]) {
            //copy target ID to class variable
            targetDetected.data = message->tags.data[0];

            geometry_msgs::Pose2D targetDetectedPos;

            float x = currentLocation.x;
            float y = currentLocation.y;

            targetDetectedPos.x = x;
            targetDetectedPos.y = y;
            targetDetectedPos.theta = currentLocation.theta;

            targetDetectedPos_01.push_back(targetDetectedPos);

            //publish detected target
            targetCollectedPublish.publish(targetDetected);

            //publish to scoring code
            targetPickUpPublish.publish(message->image);

            // goalLocation.x = currentLocation.x;
            // goalLocation.y = currentLocation.y;
            // goalLocation.theta = currentLocation.theta;

            // stateMachineState = STATE_MACHINE_TRANSFORM;
        }
    }

    //if target has not previously been detected 
    else if (targetDetected.data == -1 && self_idx == COLLECTOR) { // robot 3
        
        //check if target has not yet been collected
        if (!targetsCollected[message->tags.data[0]]) {
            //copy target ID to class variable
            targetDetected.data = message->tags.data[0];
            
            //set angle to center as goal heading
            goalLocation.theta = M_PI + atan2(currentLocation.y, currentLocation.x);
            
            //set center as goal position
            goalLocation.x = 0.0;
            goalLocation.y = 0.0;
            
            //publish detected target
            targetCollectedPublish.publish(targetDetected);

            //publish to scoring code
            targetPickUpPublish.publish(message->image);

            //switch to transform state to trigger return to center
            stateMachineState = STATE_MACHINE_TRANSFORM;
        }
    }



}

void modeHandler(const std_msgs::UInt8::ConstPtr& message) {
    currentMode = message->data;
    setVelocity(0.0, 0.0);
}

void obstacleHandler(const std_msgs::UInt8::ConstPtr& message) {
    if (message->data > 0) {
        geometry_msgs::Pose2D savedPosition;

        savedPosition.x = goalLocation.x;
        savedPosition.y = goalLocation.y;
        savedPosition.theta = goalLocation.theta;

        savedPositions.push_back(savedPosition);

        //obstacle on right side
        if (message->data == 1) {
            //select new heading 0.2 radians to the left
            goalLocation.theta = currentLocation.theta + 0.2;
        }
        
        //obstacle in front or on left side
        else if (message->data == 2) {
            //select new heading 0.2 radians to the right
            goalLocation.theta = currentLocation.theta - 0.2;
        }
                            
        //select new position 50 cm from current location
        goalLocation.x = currentLocation.x + (0.5 * cos(goalLocation.theta));
        goalLocation.y = currentLocation.y + (0.5 * sin(goalLocation.theta));

        avoiding_obstacle = true;
        
        //switch to transform state to trigger collision avoidance
        stateMachineState = STATE_MACHINE_TRANSFORM;
    }
}

void odometryHandler(const nav_msgs::Odometry::ConstPtr& message) {
    //Get (x,y) location directly from pose
    currentLocation.x = message->pose.pose.position.x;
    currentLocation.y = message->pose.pose.position.y;
    
    //Get theta rotation by converting quaternion orientation to pitch/roll/yaw
    tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    currentLocation.theta = yaw;
}

void joyCmdHandler(const geometry_msgs::Twist::ConstPtr& message) {
    if (currentMode == 0 || currentMode == 1) 
      {
    setVelocity(message->linear.x, message->angular.z);
      } 
}


void publishStatusTimerEventHandler(const ros::TimerEvent&)
{
  if(!sent_name) {
      std_msgs::String name_msg;
      name_msg.data = "I ";
      name_msg.data = name_msg.data + publishedName;
      messagePublish.publish(name_msg);
      sent_name = true;
  }

  std_msgs::String msg;
  msg.data = "CSUCI"; // 
  status_publisher.publish(msg);
}

// Safety precaution. No movement commands - might have lost contact with ROS. Stop the rover.
// Also might no longer be receiving manual movement commands so stop the rover.
void killSwitchTimerEventHandler(const ros::TimerEvent& t)
{
  // No movement commands for killSwitchTime seconds so stop the rover 
  setVelocity(0,0);
  double current_time = ros::Time::now().toSec();
  ROS_INFO("In mobility.cpp:: killSwitchTimerEventHander(): Movement input timeout. Stopping the rover at %6.4f.", current_time);
}

void targetsCollectedHandler(const std_msgs::Int16::ConstPtr& message) {
    targetsCollected[message->data] = 1;
}

void sigintEventHandler(int sig)
{
     // All the default sigint handler does is call shutdown()
     ros::shutdown();
}

void messagePasser(const std_msgs::String::ConstPtr& message)
{
    messageHandler(message, &self_idx, memberNames, publishedName, swarmSize, targetCollected, targetsDetected, targetPositions, paths, stateMachineState, goalLocation, currentLocation);
}
