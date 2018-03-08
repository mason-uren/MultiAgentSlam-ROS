#ifndef OBSTACLECONTOLLER_H
#define OBSTACLECONTOLLER_H

#include "Controller.h"
#include "Tag.h"
#include "ObstacleAssistant.h"
#include <map>
#include <cstdlib>

/*
 * Sonar has the most accurate readings at a range less than 2 meters
 * Calculated % Error:
 * ===================
 * Range:       Error:
 * -> 2m        5.31% error
 * -> 1.5m      0.125% error
 */
#define MAX_THRESH 1.5

/*
 * Web cam has max april tag detection range of 0.6604m
 * Any theoretical obstacle needs to be verified 'not a resource'
 */
#define MIN_THRESH 0.8

/*
 * With rover speed of 0.3m/s and min head on collisoin separation at 0.35m, structure size cannot exceed the value of 8.
 */
#define VECTOR_MAX 4

/*
 * Calculated max dist. lost when collision imminent between two rover, plus variance observed at 1.5m range
 */
#define DELTA 0.60894

/*
 * Reflection bounds
 */
#define R_LOW (-(M_PI) / 4)
#define R_HIGH (-(7 * M_PI) / 6)

#define L_LOW ((M_PI) / 4)
#define L_HIGH ((7 * M_PI) / 6)

#define RC_LOW (-(7 * M_PI) / 12)
#define RC_HIGH (-(4 * M_PI) / 5)

#define LC_LOW ((7 * M_PI) / 12)
#define LC_HIGH ((4 * M_PI) / 5)

#define DELAY_ITERATION 2



/*
 * Obstacle structure
 */
typedef struct {
    bool allowed;
    OBS_TYPE type;
    std::map<SONAR, ObstacleAssistant> sonar_map;
} LISTENER;

/*
 * Reflection
 */
typedef struct {
    bool should_start;
    bool should_end;
    double desired_heading;
    double reflect_angle;
} REFLECTION;

extern void logMessage(long int currentTime, string component, string message);

extern void detectionMessage(long int currentTime, string component, string message);

extern void logicMessage(long int currentTime, string component, string message);

class ObstacleController : virtual Controller {
public:
    ObstacleController();

    Result result;

    void Reset() override;

    Result DoWork() override;

    void setSonarData(float left, float center, float right);

    void setCurrentLocation(Point currentLocation);

    void setTagData(vector<Tag> tags);

    bool ShouldInterrupt() override;

    bool HasWork() override;

    void setIgnoreCenterSonar();

    void SetCurrentTimeInMilliSecs(long int time);

    void setTargetHeld();

    // Checks if a target is held and if so resets the state of the obestacle controller otherwise does nothing
    void setTargetHeldClear();

    //Asked by logiccontroller to determine if drive controller should have its waypoints cleared
    bool getShouldClearWaypoints() {
        bool tmp = clearWaypoints;
        clearWaypoints = false;
        return tmp;
    }

protected:

    void ProcessData();

    void sonarMonitor(DELAY_TYPE, float, SONAR);

//    void sonarAnalysis(ObstacleAssistant, DELAY_TYPE);
    void sonarAnalysis(DELAY_TYPE);

    void obstacleContactDir(std::map<SONAR, ObstacleAssistant>, DELAY_TYPE);

    void reflect(std::vector<double>);

    void resetObstacle(DELAY_TYPE);

    void resetDetections(DELAY_TYPE);

private:

    // Try not to run over the collection zone
    void avoidCollectionZone();

    // Try not to run into a physical object
    void avoidObstacle();

    // Are there AprilTags in the camera view that mark the collection zone
    // and are those AprilTags oriented towards or away from the camera.
    bool checkForCollectionZoneTags(vector <Tag>);

//    const float K_angular = 1.0;
    const float K_angular = 0.5; //radians a second turn rate to avoid obstacles
    const float reactivate_center_sonar_threshold = 0.8; //reactive center sonar if it goes back above this distance, assuming it is deactivated
    const int targetCountPivot = 6; ///unused variable
    const float obstacleDistancePivot = 0.2526; ///unused variable
    const float triggerDistance = 0.8;

    /*
     * Member variables
     */
    bool obstacleInterrupt; //records if obstacle has interupted
    bool obstacleDetected;  //records if an obstacle has been detected
    bool obstacleAvoided; //record if an obstacke has been avoided
    bool clearWaypoints = false;  //record if drivecontrollers waypoints should be cleared

    float left = 0; //distance on left ultrasound
    float center = 0; //distance on center ultrasound
    float right = 0; //distance on right ultrasound

    unsigned int count_left_collection_zone_tags;
    unsigned int count_right_collection_zone_tags;

    // Ignore the center sonar because we are carrying a target
    bool ignore_center_sonar = false;

    Point currentLocation;

    long int current_time;
    long int timeSinceTags;
    long int delay;

    bool targetHeld = false;
    bool previousTargetState = false;

    bool phys = false; // Physical obstacle
    bool collection_zone_seen = false; // The obstacle is the collection zone

    bool set_waypoint = false;
    bool can_set_waypoint = false;

    float camera_offset_correction = 0.020; //meters;

    string ClassName = "Obstacle Controller";

    // Print only one log message once obstacle has been encountered
    bool logInit = false;

    /*
     * Obstacle detection structure
     * (Can add more MONITORS if need be)
     */
    int stag;
    OBS_TYPE detection_declaration;
//    LISTENER obstacle_init;
//    LISTENER obstacle_stag;
    REFLECTION reflection;

    std::map<DELAY_TYPE, LISTENER> monitor_map;

    string detect_msg;

    /*
     * Turn On/Off detections
     * Default: True
     */
    bool acceptDetections;

    double x_home_tag_orientation;
};

#endif // OBSTACLECONTOLLER_H
