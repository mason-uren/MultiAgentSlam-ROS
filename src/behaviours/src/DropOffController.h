#ifndef DROPOFCONTROLLER_H
#define DROPOFCONTROLLER_H
#define HEADERFILE_H

#include "Controller.h"
#include "Tag.h"
#include "Utilities.h"
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

extern void logMessage(long int currentTime, string component, string message);

extern void logicMessage(long int currentTime, string component, string message);
extern void tagMessage(vector <Tag> tags);
extern void dropOffMessage(string component, string message);

class DropOffController : virtual Controller {
public:
    DropOffController();

    ~DropOffController();

    void Reset() override;

    Result DoWork() override;

    bool ShouldInterrupt() override;

    bool HasWork() override;

    bool IsChangingMode();

    void SetCenterLocation(Point center);

    void SetCurrentLocation(Point current);

    void SetTargetPickedUp();

    void SetBlockBlockingUltrasound(bool blockBlock);

    void SetTargetData(vector<Tag> tags);

    bool HasTarget() { return targetHeld; }

    float GetSpinner() { return spinner; }

    void UpdateData(vector<Tag> tags);

    void SetCurrentTimeInMilliSecs(long int time);

    Point closestAnchor(Point current);



private:

    void ProcessData();
    void Align();
    void DropCube();
    void WaypointNav();
    void SearchForHome();
    void DeliverCube();
    void BackUp();
    void AltDeliver();

    /**
     * Get the index of the center tag closest to the camera
     * @param tags Vector of tags to search
     * @return The index of the closest tag, or -1 if none found
     */
    int getClosestCenterTagIdx(const vector<Tag> &tags);

    /**
     * Gets the distance from the camera to a tag
     * @param tag The tag to get the distance to
     * @return The distance (presumably in meters)
     */
    double tagDistanceFromCamera(const Tag &tag);

    //Constants

    const float cameraOffsetCorrection = 0.020; //meters
    const float centeringTurnRate = 0.15; //radians
    const int centerTagThreshold = 8;
    const int lostCenterCutoff = 4; //seconds before giving up on drop off beacuse center cannot be seen anymore
    const float collectionPointVisualDistance = 0.2; //in meters
    const float initialSpinSize = 0.05; //in meters aka 10cm
    const float spinSizeIncrement = 0.50; //in meters
    const float searchVelocity = 0.15; //in meters per second
    const float dropDelay = 0.5; //delay in seconds for dropOff
    const double centerClearedDistanceThreshold = .25; // Distance we must be from the center to consider it cleared
    const double minimumBackupThreshold = 2.5; // Minimum time to spend backing up
    const double deliveryTimeThreshold = 0.4; //Amount of time to drive forward after no tags are seen

    //New globals
    double tagYaw; //yaw of closest tag
    double prevYaw;
    double centerYaw;
    double alignAngleSumLeft;
    double alignAngleSumRight;
    bool isAligned; //aligned to tag based on 0 orientation of home tag yaw
    bool firstAlign;
    bool firstReAlign;
    bool alternateDeliver;
    bool edgeCase;
    bool homeFound; //to prevent going into waypoint or home search during delivery
    bool startDeliverTimer; //trigger for starting delivery timer
    bool noLeft; //triggers AltAlignmentTagCheck
    bool noRight; //triggers AltAlignmentTagCheck
    bool altAlignEdge;
    bool altAlignCorner;

    //Instance Variables

    /*
       *  Timers and Accumulators
       */

    //keep track of progression around a circle when driving in a circle
    float spinner;

    //Timer for return code (dropping the cube in the center)- used for timerTimeElapsed
    long int returnTimer;

    //Time since last exceeding the tag threshold
    long int lastCenterTagThresholdTime;

    //Previous tag count
    int prevCount;


    /*
       *  Cached External Information
       */

    //Count of tags on the left and right, respectively
    int countLeft;
    int countRight;
    int tagCount;
    int countCenter;

    // Distance from the camera of the closest tag
    double closestTagDistance;

    //Center and current locations as of the last call to setLocationData
    Point centerLocation;
    Point currentLocation;

    //Time since modeTimer was started, in seconds
    float timerTimeElapsed;

    float deliverTimer;

    float alignTimer;
    float realignTimer;

    //The amount over initialSpinSize we've gotten to
    float spinSizeIncrease;

    /*
       *  Flags
       */

    //Flag indicating that a target has been picked up and is held
    bool targetHeld;

    //Flag indicating that we're in the center
    bool reachedCollectionPoint;

    //Flag indicating that we're driving in a circle to find the nest
    bool circularCenterSearching;

    //Flag for when we are entering the center circle
    bool centerApproach;

    //we have seen enough central collection tags to be certain we are either in or driving towards the nest.
    bool seenEnoughCenterTags;

    //Flag to indicate a switch to precision driving
    bool isPrecisionDriving;

    //Flag to indicate that we're starting to follow waypoints
    bool startWaypoint;

    Tag average_center_tag;
    float blockDistance;
    float startDeliveryTime;
    float deliveryTimer;

    Result result;

    //current ROS time from the RosAdapter
    long int current_time;

    bool interrupt = false;
    bool precisionInterrupt = false;
    bool finalInterrupt = false;
    bool first_center = true;

    string ClassName = "DropOff Controller";

    float gps_chase_timer = 0.0;

};

#endif // end header define
