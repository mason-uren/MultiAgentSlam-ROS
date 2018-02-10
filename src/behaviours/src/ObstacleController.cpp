#include "ObstacleController.h"
#include "ObstacleAssistant.h"

ObstacleController::ObstacleController() {
    obstacleAvoided = true;
    obstacleDetected = false;
    obstacleInterrupt = false;
    result.PIDMode = CONST_PID; //use the const PID to turn at a constant speed

    /*
     * Create 'OBSTACLE' structures to contain all evaluations and obstacle decision of the sonar readings.
     * Contains:
     * -> 'type' of obstacle
     * -> 'sonar_map' of feedback from each sensor
     */
    // INIT obstacle detection
    this->obstacle_init.type = NO_OBSTACLE;
    this->obstacle_init.delay = INIT;
    this->obstacle_init.allowed = true;
    this->obstacle_init.sonar_map = {
            {LEFT, ObstacleAssistant(LEFT)},
            {CENTER, ObstacleAssistant(CENTER)},
            {RIGHT, ObstacleAssistant(RIGHT)}
    };
    // STAG obstacle detection
    this->obstacle_stag.type = NO_OBSTACLE;
    this->obstacle_stag.delay = STAG;
    this->obstacle_stag.allowed = false;
    this->obstacle_stag.sonar_map = {
            {LEFT, ObstacleAssistant(LEFT)},
            {CENTER, ObstacleAssistant(CENTER)},
            {RIGHT, ObstacleAssistant(RIGHT)}
    };


}


//note, not a full reset as this could cause a bad state
//resets the interupt and knowledge of an obstacle or obstacle avoidance only.
void ObstacleController::Reset() {
    obstacleAvoided = true;
    obstacleDetected = false;
    obstacleInterrupt = false;
    delay = current_time;
}

// Avoid crashing into objects detected by the ultraound
void ObstacleController::avoidObstacle() {

    //always turn left to avoid obstacles
    if (right < 0.8 || center < 0.8 || left < 0.8) {
        result.type = precisionDriving;

        result.pd.cmdAngular = -K_angular;

        result.pd.setPointVel = 0.0;
        result.pd.cmdVel = 0.0;
        result.pd.setPointYaw = 0;
    }
}

// A collection zone was seen in front of the rover and we are not carrying a target
// so avoid running over the collection zone and possibly pushing cubes out.
void ObstacleController::avoidCollectionZone() {

    result.type = precisionDriving;

    result.pd.cmdVel = 0.0;

    // Decide which side of the rover sees the most april tags and turn away
    // from that side
    if (count_left_collection_zone_tags < count_right_collection_zone_tags) {
        result.pd.cmdAngular = K_angular;
    } else {
        result.pd.cmdAngular = -K_angular;
    }

    result.pd.setPointVel = 0.0;
    result.pd.cmdVel = 0.0;
    result.pd.setPointYaw = 0;
}


Result ObstacleController::DoWork() {
    std::cout << "ObstacleController -> DoWork" << std::endl;

    clearWaypoints = true;
    set_waypoint = true;
    result.PIDMode = CONST_PID;
    string msg;

    if (obstacleDetected) {
        /*
         * Update "/logger" publisher -> Starting avoidance
         */
        if (!logInit) {
            msg = "Starting obstacle avoidance routine.";
            logMessage(current_time, "ObstacleController", msg);
            logInit = true;
        }

        // The obstacle is an april tag marking the collection zone
        if (collection_zone_seen) {
            avoidCollectionZone();
        } else {
            avoidObstacle();
        }
    }


    //if an obstacle has been avoided
    if (can_set_waypoint) {
        /*
         * Update "/logger" publisher -> Exiting avoidance
         */
        msg = "Exiting obstacle avoidance.";
        logMessage(current_time, "ObstacleController", msg);
        logInit = false;

        can_set_waypoint = false; //only one waypoint is set
        set_waypoint = false;
        clearWaypoints = false;

        result.type = waypoint;
        result.PIDMode = FAST_PID; //use fast pid for waypoints
        Point forward;            //waypoint is directly ahead of current heading
        forward.x = currentLocation.x + (0.5 * cos(currentLocation.theta));
        forward.y = currentLocation.y + (0.5 * sin(currentLocation.theta));
        result.wpts.waypoints.clear();
        result.wpts.waypoints.push_back(forward);
    }

    return result;
}


void ObstacleController::setSonarData(float sonarleft, float sonarcenter, float sonarright) {
    left = sonarleft;
    right = sonarright;
    center = sonarcenter;

    ProcessData();
}

void ObstacleController::setCurrentLocation(Point currentLocation) {
    this->currentLocation = currentLocation;
}

void ObstacleController::ProcessData() {
    //timeout timer for no tag messages
    //this is used to set collection zone seen to false beacuse
    //there is no report of 0 tags seen
    long int Tdifference = current_time - timeSinceTags;
    float Td = Tdifference / 1e3;
    if (Td >= 0.5) {
        collection_zone_seen = false;
        phys = false;
        if (!obstacleAvoided) {
            can_set_waypoint = true;
        }
    }

    //If we are ignoring the center sonar
    if (ignore_center_sonar) {
        //If the center distance is longer than the reactivation threshold
        if (center > reactivate_center_sonar_threshold) {
            //currently do not re-enable the center sonar instead ignore it till the block is dropped off
            //ignore_center_sonar = false; //look at sonar again beacuse center ultrasound has gone long
        } else {
            //set the center distance to "max" to simulated no obstacle
            center = 3;
        }
    } else {
        //this code is to protect against a held block causing a false short distance
        //currently pointless due to above code
        if (center < 3.0) {
            result.wristAngle = 0.7;
        } else {
            result.wristAngle = -1;
        }
    }


    // TODO: CURRENT WORk
    // Each iteration through should have to recalc detection
    this->obstacle_init.type = NO_OBSTACLE;
    this->obstacle_stag.type = NO_OBSTACLE;
    // Delay the start of the second set of monitors
    if (this->obstacle_stag.allowed) {
        this->obstacle_stag.allowed = true;
    }
    else {
        this->stag++;
    }

    /*
     * METHODOLODY
     * 1) Is below 'MAX_THRESH'?
     * 2) Check is any of the monitors are at capacity
     * 3) Is valid detection? Is below 'MIN_THRESH'
     * 4) Determine avoidance type and measures
     * 5) Verify that MONITORS agree, then assign agreement detection
     */


    /* 1)
     * Monitor the detection values that fall below our 'MAX_THRESH'
     * INIT: should always be running from the start
     * STAG: needs to wait two iterations before starting, then should always be running
     */
    if (left <= MAX_THRESH) {
        sonarMonitor(this->obstacle_init, left, LEFT); // INIT
        sonarMonitor(this->obstacle_stag, left, LEFT); // STAG
    }
    if (center <= MAX_THRESH) {
        sonarMonitor(this->obstacle_init, center, CENTER); // INIT
        sonarMonitor(this->obstacle_stag, center, CENTER); // STAG
    }
    if (right <= MAX_THRESH) {
        sonarMonitor(this->obstacle_init, right, RIGHT); // INIT
        sonarMonitor(this->obstacle_stag, right, RIGHT); // STAG
    }

    /* 2)
     * Check if any of the monitors are at capacity
     */
    for (auto assistant : this->obstacle_init.sonar_map) { // INIT
        if (assistant.second.detections.init_detection) {
            sonarAnalysis(assistant.second, INIT);
        }
    }
    if (this->obstacle_stag.stag_started) { // STAG
        for (auto assistant: this->obstacle_stag.sonar_map) {
            if (assistant.second.detections.init_detection) {
                sonarAnalysis(assistant.second, STAG);
            }
        }
    }

    /* 3)
     * If any of the monitors are valid, check if they have cross the 'MIN_THRESH'.
     * Populate temporary map with acceptable detections.
     */
    auto *INIT_sonar_map = new std::map<SONAR, ObstacleAssistant>(); // INIT
    for (auto assistant : this->obstacle_init.sonar_map) {
        if (assistant.second.detections.good_detection) {
            SONAR type = assistant.first;
            INIT_sonar_map->insert(std::pair<SONAR, ObstacleAssistant>(type, assistant.second));
        }
    }
    auto *STAG_sonar_map = new std::map<SONAR, ObstacleAssistant>(); // STAG
    for (auto assistant : this->obstacle_stag.sonar_map) {
        if (assistant.second.detections.good_detection) {
            SONAR type = assistant.first;
            STAG_sonar_map->insert(std::pair<SONAR, ObstacleAssistant>(type, assistant.second));
        }
    }

    /* 4)
     * Iterate through temporary sonar maps and determine correct avoidance type and measures
     */
    if (!INIT_sonar_map->empty()) { // INIT
        obstacleContactDir(*INIT_sonar_map, INIT);
        // Drop temp sonar mapping
        INIT_sonar_map->clear();
    }
    if (!STAG_sonar_map->empty()) { // STAG
        obstacleContactDir(*STAG_sonar_map, STAG);
        STAG_sonar_map->clear();
    }

    /* 5)
     * Only accept a detection if both monitors agree that there is an obstacle. If one flags but not the other
     * then there is the possibility that it's still a false detection.
     * NOTE: There shouldn't be any obstacle impeeding the rovers path 0.2 secs after its been
     * turned on, so this statement should be caught by 'obstacle_init' and skipped before
     * the initialization of 'obstacle_stag'
     */
    if (this->obstacle_init.type != NO_OBSTACLE && (this->obstacle_init.type == this->obstacle_stag.type)) { // <--- TODO: try this line first
//    if (this->obstacle_init.type != NO_OBSTACLE && this->obstacle_stag.type != NO_OBSTACLE) {
//        this->detection_declaration = this->obstacle_init.type;
        phys = true;
        timeSinceTags = current_time;

        obstacleDetected = true;
        obstacleAvoided = false;
        can_set_waypoint = false;

    }
    else {
        obstacleAvoided = true;
    }

    std::cout << "Obstacle Type Init --->> " << this->obstacle_init.type << std::endl;
    std::cout << "Obstacle Type Stag --->> " << this->obstacle_stag.type << std::endl;



    // TODO: adjust 'triggerDistance' to the current max tag idenfication range (~0.6m)
    // TODO: better evaluate sonar data that is passed
    //if any sonar is below the trigger distance set physical obstacle true
//    if (left < triggerDistance || right < triggerDistance || center < triggerDistance) {
//        phys = true;
//        timeSinceTags = current_time;
//    }

    //if physical obstacle or collection zone visible
    // TODO: this is where a detection is flagged and passed to either 'avoidCollectionZone' or 'avoidObstacle'
//    if (collection_zone_seen || phys) {
//        obstacleDetected = true;
//        obstacleAvoided = false;
//        can_set_waypoint = false;
//    } else {
//        obstacleAvoided = true;
//    }
}

// Report April tags seen by the rovers camera so it can avoid
// the collection zone
// Added relative pose information so we know whether the
// top of the AprilTag is pointing towards the rover or away.
// If the top of the tags are away from the rover then treat them as obstacles. 
void ObstacleController::setTagData(vector<Tag> tags) {
    collection_zone_seen = false;
    count_left_collection_zone_tags = 0;
    count_right_collection_zone_tags = 0;

    // this loop is to get the number of center tags
    if (!targetHeld) {
        for (int i = 0; i < tags.size(); i++) { //redundant for loop
            if (tags[i].getID() == 256) {

                collection_zone_seen = checkForCollectionZoneTags(tags);
                timeSinceTags = current_time;
            }
        }
    }
}

bool ObstacleController::checkForCollectionZoneTags(vector<Tag> tags) {

    for (auto &tag : tags) {

        // Check the orientation of the tag. If we are outside the collection zone the yaw will be positive so treat the collection zone as an obstacle.
        //If the yaw is negative the robot is inside the collection zone and the boundary should not be treated as an obstacle.
        //This allows the robot to leave the collection zone after dropping off a target.
        if (tag.calcYaw() > 0) {
            // checks if tag is on the right or left side of the image
            if (tag.getPositionX() + camera_offset_correction > 0) {
                count_right_collection_zone_tags++;

            } else {
                count_left_collection_zone_tags++;
            }
        }

    }


    // Did any tags indicate that the robot is inside the collection zone?
    return count_left_collection_zone_tags + count_right_collection_zone_tags > 0;

}

//obstacle controller should inrerupt is based upon the transition from not seeing and obstacle to seeing an obstacle
bool ObstacleController::ShouldInterrupt() {

    //if we see and obstacle and havent thrown an interrupt yet
    if (obstacleDetected && !obstacleInterrupt) {
        obstacleInterrupt = true;
        return true;
    } else {
        //if the obstacle has been avoided and we had previously detected one interrupt to change to waypoints
        if (obstacleAvoided && obstacleDetected) {
            Reset();
            return true;
        } else {
            return false;
        }
    }
}

bool ObstacleController::HasWork() {
    //there is work if a waypoint needs to be set or the obstacle hasnt been avoided
    if (can_set_waypoint && set_waypoint) {
        return true;
    }

    return !obstacleAvoided;
}

//ignore center ultrasound
void ObstacleController::setIgnoreCenterSonar() {
    ignore_center_sonar = true;
}

void ObstacleController::setCurrentTimeInMilliSecs(long int time) {
    current_time = time;
}

void ObstacleController::setTargetHeld() {
    targetHeld = true;

    //adjust current state on transition from no cube held to cube held
    if (previousTargetState == false) {
        obstacleAvoided = true;
        obstacleInterrupt = false;
        obstacleDetected = false;
        previousTargetState = true;
    }
}

void ObstacleController::setTargetHeldClear() {
    //adjust current state on transition from cube held to cube not held
    if (targetHeld) {
        Reset();
        targetHeld = false;
        previousTargetState = false;
        ignore_center_sonar = false;
    }
}

//TODO: implement correction angle calculator based on acceptable values
//Need to make sure to only pass good sonar detections
/**
 * If acceptedable detections cross 'MIN_THRESH' mark the direction of the obstacle
 * @param accepted_sonar : temporary map of accepted sonar vectors
 * @param delay : which monitor are we editing
 */
void ObstacleController::obstacleContactDir(std::map<SONAR, ObstacleAssistant> accepted_sonar, DELAY_TYPE delay) {
    double x;
    double y;
    double alpha;
    this->obstacle_init.type = NO_OBSTACLE;
    this->obstacle_stag.type = NO_OBSTACLE;
    /*
     * Grab the accepted detection from the map and verify they
     * are below our 'MIN_THRESH'.
     */
    ObstacleAssistant *left = NULL;
    ObstacleAssistant *center = NULL;
    ObstacleAssistant *right= NULL;
    for (auto assistant : accepted_sonar) {
        if (assistant.second.detections.smallest_detection < MIN_THRESH) {
            switch (assistant.first) {
                case LEFT:
                    left = &assistant.second;
                    break;
                case CENTER:
                    center = &assistant.second;
                    break;
                case RIGHT:
                    right = &assistant.second;
                    break;
                default:
                    std::cout << "OBSTACLE_CONTROLLER: hit default in 'ObstacleController::obstacleContactDir()" << std::endl;
                    break;
            }
        }
        else {
            // Remove sonar that has not passed below our 'MIN_THRESH'
            accepted_sonar.erase(assistant.first);
            // Restart respective sonar monitor
            switch (delay) {
                case INIT:
                    this->obstacle_init.sonar_map.at(assistant.first).monitor->clear();
                    break;
                case STAG:
                    this->obstacle_stag.sonar_map.at(assistant.first).monitor->clear();
                    break;
                default:
                    std::cout << "OBSTACLE_CONTROLLER: hit default in INIT STAG differentiation" << std::endl;
                    break;
            }

        }
    }

    /*
     * Check if there are any good detections
     */
    switch (delay) {
        case INIT:
            if (!accepted_sonar.empty()) {
                if (left && center && right) {
                    this->obstacle_init.type = OBS_CENTER;
                }
                else if (center && left) {
                    this->obstacle_init.type = OBS_LEFT_CENTER;
                }
                else if (center && right) {
                    this->obstacle_init.type = OBS_RIGHT_CENTER;
                }
                else if (center) {
                    this->obstacle_init.type = OBS_CENTER;
                }
                else if (left) {
                    this->obstacle_init.type = OBS_LEFT;
                }
                else if (right) {
                    this->obstacle_init.type = OBS_RIGHT;

                }
            }
                // Reset 'obstacle' monitors and detections
            else {
                resetObstacle(INIT);
            }
            break;
        case STAG:
            if (!accepted_sonar.empty()) {
                if (left && center && right) {
                    this->obstacle_stag.type = OBS_CENTER;
                }
                else if (center && left) {
                    this->obstacle_stag.type = OBS_LEFT_CENTER;
                }
                else if (center && right) {
                    this->obstacle_stag.type = OBS_RIGHT_CENTER;
                }
                else if (center) {
                    this->obstacle_stag.type = OBS_CENTER;
                }
                else if (left) {
                    this->obstacle_stag.type = OBS_LEFT;
                }
                else if (right) {
                    this->obstacle_stag.type = OBS_RIGHT;

                }
            }
                // Reset 'obstacle' monitors and detections
            else {
                resetObstacle(STAG);
            }
            break;
        default:
            std::cout << "OBSTACLE_CONTROLLER: hit default" << std::endl;
            break;

    }

}

/**
 * Iterates of the passed structure and verifies detection are of acceptable range.
 * Note: Acceptable values are messured against 'DELTA' (calculated max dist. the rover can cover in 1 sec.)
 * @param assistant : the passed obstacle assistant (type, detections, monitor)
 * @param delay : which monitor are we editing
 */
void ObstacleController::sonarAnalysis(ObstacleAssistant assistant, DELAY_TYPE delay) {
    float prev;
    float curr;
    bool has_begun = false;
    SONAR sonar = assistant.type;
    // check to make sure the passed structure has been populated
    if (!assistant.monitor->empty()) {
        for (auto detectionRange : *assistant.monitor) {
            if (!has_begun) {
                curr = detectionRange;
                has_begun = true;
            } else {
                prev = curr;
                curr = detectionRange;
                double diff = std::fabs(prev - curr);
                if (diff < DELTA) {
                    switch (delay) {
                        case INIT:
                            this->obstacle_init.sonar_map.at(sonar).detections.good_detection = true;
                            this->obstacle_init.sonar_map.at(sonar).detections.smallest_detection = curr;
                            break;
                        case STAG:
                            this->obstacle_stag.sonar_map.at(sonar).detections.good_detection = true;
                            this->obstacle_stag.sonar_map.at(sonar).detections.smallest_detection = curr;
                    }
    //                    last_detect = curr;
                    // TESTING: noticed false detections
    //                    if (curr > 0.06) {
    //                        last_detect = curr;
    //                    }
    //                    else {
    //                        bad_detection = true;
    //                        last_detect = -1;
    //                    }
                }
                else { // Throw out bad vector
                    switch (delay) {
                        case INIT:
                            this->obstacle_init.sonar_map.at(sonar).detections.good_detection = false;
                            this->obstacle_init.sonar_map.at(sonar).monitor->clear(); // Remove bad vector
                            break;
                        case STAG:
                            this->obstacle_stag.sonar_map.at(sonar).detections.good_detection = false;
                            this->obstacle_stag.sonar_map.at(sonar).monitor->clear(); // Remove bad vector
                            break;
                    }
                    break;
                }
            }
        }
    }
}

/**
 * Adds passed values of sonar detections to respective structures, then checks
 * to make sure structure doesn't exceed specified size 'VECTOR_MAX'
 * @param obstacle : the entire obstacle structure (INIT, STAG, etc) <-- if more are added
 * @param range : the distance of the respective detection
 * @param sonar : which sensor are we referencing
 */
void ObstacleController::sonarMonitor(OBSTACLE obstacle, float range, SONAR sonar) {
    if (obstacle.allowed) {
        obstacle.sonar_map.at(sonar).monitor->push_back(range);
        if (obstacle.sonar_map.at(sonar)->size() >= VECTOR_MAX) {
            switch (which) {
                case INIT:
                    this->obstacle_init.sonar_map.at(sonar).detections.init_detection = true;
                    break;
                case STAG:
                    this->obstacle_stag.sonar_map.at(sonar).detections.init_detection = true;
                    break;
            }
        }
    }
}

/**
 * Reset the ObstacleAssistant structure when no good detection are made
 * @param delay_type : which 'assistant' is being passed
 */
void ObstacleController::resetObstacle(DELAY_TYPE delay_type) {
    switch (delay_type) {
        case INIT:
            this->obstacle_init.type = NO_OBSTACLE;
            this->obstacle_init.sonar_map = {
                    {LEFT, ObstacleAssistant(LEFT)},
                    {CENTER, ObstacleAssistant(CENTER)},
                    {RIGHT, ObstacleAssistant(RIGHT)}
            };
            break;
        case STAG:
            this->obstacle_stag.type = NO_OBSTACLE;
            this->obstacle_stag.sonar_map = {
                    {LEFT, ObstacleAssistant(LEFT)},
                    {CENTER, ObstacleAssistant(CENTER)},
                    {RIGHT, ObstacleAssistant(RIGHT)}
            };
            break;
        default:
            std::cout << "OBSTACLE_CONTROLLER: hit default in resetObstacle " << std::endl;
            break;
    }
}


