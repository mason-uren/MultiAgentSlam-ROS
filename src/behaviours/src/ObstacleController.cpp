#include "ObstacleController.h"
#include <cmath>
#include <angles/angles.h>


ObstacleController::ObstacleController() {
    /*
     * Turn sonar On/Off
     */
    this->acceptDetections = true;

    /*
     * TODO: For testing purposes
     */
    this->controller = OBSTACLE;

    collection_zone_seen = false;
    obstacleAvoided = true;
    obstacleDetected = false;
    obstacleInterrupt = false;
    result.PIDMode = CONST_PID; //use the const PID to turn at a constant speed

    /*
     * Create a 'monitor_map' of LISTENER objects using the DELAY_TYPE as a key.
     * Structure is setup to allow multiple detection monitors to run in parallel.
     */
    std::map<SONAR, ObstacleAssistant> assistant_map = {
            {LEFT, ObstacleAssistant(LEFT)},
            {CENTER, ObstacleAssistant(CENTER)},
            {RIGHT, ObstacleAssistant(RIGHT)}
    };
    LISTENER listener = {true, NO_OBSTACLE, assistant_map};
    this->monitor_map = {
            {INIT, listener},
            {STAG, listener}
    };
    this->stag = 0;

    /*
     * Create reflection structure
     */
    this->reflection.should_start = true;
    this->reflection.should_end = false;
}


//note, not a full reset as this could cause a bad state
//resets the interupt and knowledge of an obstacle or obstacle avoidance only.
void ObstacleController::Reset() {
    std::cout << "OBSTACLE CONTROLLER: reset" << std::endl;
    obstacleAvoided =  true;
    obstacleDetected = false;
    obstacleInterrupt = false;
    collection_zone_seen = false;
    phys = false;
    can_set_waypoint = true;
    this->reflection.should_start = true;
    this->reflection.should_end = false;
    this->detection_declaration = NO_OBSTACLE;
    for (auto monitor : monitor_map) {
        this->resetObstacle(monitor.first);
    }


    delay = current_time;
}

// Avoid crashing into objects detected by the ultraound
void ObstacleController::avoidObstacle() {
    logicMessage(current_time, ClassName, __func__);

    result.type = precisionDriving;
    result.pd.setPointVel = 0.0;
    result.pd.cmdVel = 0.0;
    result.pd.setPointYaw = 0;

    /*
     * Based on detection location reflect off of obstacle in
     * opposing direction.
     * NOTE: only initial detection is recorded by 'reflect' function
     */
    std::cout << "OBSTACLE CONTROLLER: avoidObstacle" << std::endl;
    switch (this->detection_declaration) {
        case OBS_LEFT:
            this->reflect({R_LOW, R_HIGH});
            result.pd.cmdAngular = -K_angular;
        case OBS_LEFT_CENTER: case OBS_CENTER: // OBS_CENTER left in but currently will not be set
            this->reflect({RC_LOW, RC_HIGH});
            result.pd.cmdAngular = -K_angular; // Turn Right
            break;
        case OBS_RIGHT:
            this->reflect({L_LOW, L_HIGH});
            result.pd.cmdAngular = K_angular;
            break;
        case OBS_RIGHT_CENTER:
            this->reflect({LC_LOW, LC_HIGH});
            result.pd.cmdAngular = K_angular; // Turn Left
            break;
        default:
            std::cout << "OBSTACLE_CONTROLLER: avoidObstacle(), hit default" << std::endl;
            break;
    }
}

// A collection zone was seen in front of the rover and we are not carrying a target
// so avoid running over the collection zone and possibly pushing cubes out.
void ObstacleController::avoidCollectionZone() {
    logicMessage(current_time, ClassName, __func__);

    std::cout << "HOME DETECTED: avoidCollectionZone()" << std::endl;

    result.type = precisionDriving;
    result.pd.cmdVel = 0.0;

    /*
     * Decide which side of the rover is closer to april tags
     * Left (Positive) | Right (Negative)
     */
    if (this->x_home_tag_orientation > 0) { // Home tags on left
         this->reflect({LC_LOW, LC_HIGH});
         result.pd.cmdAngular = K_angular; // Turn right
         std::cout << "LEFT TURN!! LEFT TURN!!" << std::endl;
    } else { // Home tages on right
         this->reflect({RC_LOW, RC_HIGH});
         result.pd.cmdAngular = -K_angular; // Turn Left
         std::cout << "RIGHT TURN!! RIGHT TURN!!" << std::endl;
    }
    result.pd.setPointVel = 0.0;
    result.pd.cmdVel = 0.0;
    result.pd.setPointYaw = 0;
}


Result ObstacleController::DoWork() {
    logicMessage(current_time, ClassName, __func__);

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

        /*
         * The obstacle is an april tag marking the collection zone
         * HOME retains highest priority since it is checked first
         */
        /*
         * When the rover is not holding a cube, reflect off of obstacles
         */
        if (!targetHeld) {
            std::cout << "Do Work. Detections is " << this->detection_declaration << std::endl;
            if (this->detection_declaration == HOME) {
                avoidCollectionZone();
            } else {
                avoidObstacle();
            }
        }
        /*
         * When the rover is holding a cube, traverse obstacles
         */
        else {
            std::cout << "Holding Cube" << std::endl;
            std::cout << "Begin traversal" << std::endl;
            this->traversal();
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
        // TODO: obstacle traversal
        Point lastLocation = GetLastCubeLocation();

        if (targetHeld) {
            result.type = waypoint;
            result.waypoints.clear();
            lastLocation.x = 0; //this can be changed, needed to make sure cluster collection wasnt dying here
            lastLocation.y = 0;
            lastLocation.theta = 0;
            result.waypoints.push_back(lastLocation);
            cout << "target held go home 0,0,0 " << endl;
        } else if (lastLocation.x != 0 && lastLocation.y != 0) {
            cout << "in obstacle 171: " << lastLocation.x << endl;
            result.type = waypoint;
            result.waypoints.clear();
            result.waypoints.push_back(lastLocation);
        } else {
            result.type = vectorDriving;
        }

        result.PIDMode = FAST_PID; //use fast pid for waypoints
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
    //If we are ignoring the center sonar
    if (ignore_center_sonar) {
        //If the center distance is longer than the reactivation threshold
//        if (center > reactivate_center_sonar_threshold) {
        if (center > MIN_THRESH) {
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

    /**
     * Each iteration through should have to recalc detection.
     * Final detection should be checked if it is allowed to end.
     */
    for (auto monitor : monitor_map) {
        this->resetDetections(monitor.first);
    }

    /*
     * Stagger the start of the declared monitors
     */
    for (auto monitor : this->monitor_map) {
        // Every even iteration will start another monitor if one is available and hasn't been started
        if (!monitor.second.allowed && ((int) monitor.first + this->stag) % DELAY_ITERATION == 0) {
            monitor.second.allowed = true;
            this->monitor_map.at(monitor.first) = monitor.second;
            break;
        }
        else if (this->stag < this->monitor_map.size()){
            this->stag++;
        }
    }

    /*
     * DETECTION METHODOLOGY
     * 1) Is below 'MAX_THRESH'?
     * 2) Check is any of the monitors are at capacity
     * 3) Is valid detection? Is below 'MIN_THRESH'
     * 4) Determine avoidance type and measures
     * 5) Verify that MONITORS agree, then assign agreement detection
     */


    /* 1)
     * Monitor the detection values that fall below our 'MAX_THRESH'
     * Sets 'init_detection' to 'true' if sonar monitor is at capacity.
     */
    for (auto monitor : this->monitor_map) {
        if (left <= MAX_THRESH) {
            this->sonarMonitor(monitor.first, left, LEFT);
        }
        if (center <= MAX_THRESH) {
            this->sonarMonitor(monitor.first, center, CENTER);
        }
        if (right <= MAX_THRESH) {
            this->sonarMonitor(monitor.first, right, RIGHT);
        }
    }

    /* 2)
     * Check if any of the monitors are at capacity.
     */
    for (auto monitor : this->monitor_map) {
        if (monitor.second.allowed) {
            this->sonarAnalysis(monitor.first);
        }
    }

    /* 3)
     * If any of the monitors are valid, check if they have cross the 'MIN_THRESH'.
     * Populate temporary map with acceptable detections. Then clear respective monitor so new detections can be made.
     */
    auto *accepted_monitor_map = new std::map<DELAY_TYPE, LISTENER>();
    for (auto monitor : this->monitor_map) {
        if (monitor.second.allowed) {
            auto *accepted_sonar_map = new std::map<SONAR, ObstacleAssistant>();
            for (auto assistant : monitor.second.sonar_map) {
                if (assistant.second.detections.good_detection) {
                    accepted_sonar_map->insert(std::pair<SONAR, ObstacleAssistant>(assistant.first, assistant.second));
                    // Reset acceptable monitor after it has been copied
                    assistant.second.monitor->clear();
                }
            }
            // Add to values temporary map
            monitor.second.sonar_map = *accepted_sonar_map;
            accepted_sonar_map->clear();
            accepted_monitor_map->insert(std::pair<DELAY_TYPE, LISTENER>(monitor.first, monitor.second));
        }
    }

    /* 4)
     * Iterate through temporary sonar maps and determine correct avoidance type and measures
     */
    for (auto monitor : *accepted_monitor_map) {
        if (!monitor.second.sonar_map.empty()) {
            obstacleContactDir(monitor.second.sonar_map, monitor.first);
            // Drop temp monitor mapping
            accepted_monitor_map->clear();
        }
    }

    /* 5)
     * Only accept a detection if all monitors agree that there is an obstacle. If one flags but not the other
     * then there is the possibility that it's still a false detection.
     */
    bool can_see_obs = false;
    for (auto monitor : this->monitor_map) {
        if (monitor.second.allowed && monitor.second.type != NO_OBSTACLE) {
            can_see_obs = true;
        }
        else {
            can_see_obs = false;
        }
    }
    // Check if we're listening to detections
    if (can_see_obs && acceptDetections && this->reflection.should_start) {
        // Declare detection, but `HOME` detection should always have priority TODO:
        if (this->detection_declaration != HOME) {
            this->detection_declaration = this->monitor_map.at(INIT).type;
            std::cout << "LISTENER CONTROLLER: processData() detection made: " << this->detection_declaration
                      << std::endl;
        }
        for (auto monitor : this->monitor_map) {
            resetObstacle(monitor.first);
        }
    }

    // Set flow control variables
    std::cout << "Detection Dec: " << this->detection_declaration << std::endl;
    if (this->reflection.should_start) {
        if (this->detection_declaration != NO_OBSTACLE) {
            phys = true;
            obstacleDetected = true;
            obstacleAvoided = false;
            can_set_waypoint = false;
        }
    } else if (this->reflection.should_end) {
        std::cout << "SHOULD END: reset detection besides HOME" << std::endl;
        // Verify that we've completed the reflection off the obstacle
        obstacleAvoided = true;
        phys = false;
        obstacleDetected = false;
        can_set_waypoint = true;
        this->reflection.should_start = true;
        this->reflection.should_end = false;
        if (this->detection_declaration == HOME) {
            std::cout << "RESET HOME" << std::endl;
            this->detection_declaration = NO_OBSTACLE;
            collection_zone_seen = false;
        }
    }

    /*
     * Detection Logger
     */
    if (detection_declaration != NO_OBSTACLE) {
        detect_msg = "Detection: " + to_string(this->detection_declaration);
        detectionMessage(current_time, "ObstacleController", detect_msg);
    }
}

// Report April tags seen by the rovers camera so it can avoid
// the collection zone
// Added relative pose information so we know whether the
// top of the AprilTag is pointing towards the rover or away.
// If the top of the tags are away from the rover then treat them as obstacles.
void ObstacleController::setTagData(vector <Tag> tags) {
    std::cout << "Inside setTageData()" << std::endl;
    if (!collection_zone_seen) {
        std::cout << "Collection zone not seen" << std::endl;
        count_left_collection_zone_tags = 0;
        count_right_collection_zone_tags = 0;
        x_home_tag_orientation = 0; // Todo:

        if (!targetHeld) {
            collection_zone_seen = checkForCollectionZoneTags(tags);
            if (collection_zone_seen) {
                std::cout << "HOME HOME HOME HOME" << std::endl;
                this->detection_declaration = HOME;
                this->reflection.should_start;
            }
        }
    }
}

bool ObstacleController::checkForCollectionZoneTags(vector<Tag> tags) {
    bool home_seen = false;
    for (auto &tag : tags) {

        if (tag.getID() == 256 && tag.calcYaw() > 0) {
            this->x_home_tag_orientation += tag.getPositionX() + camera_offset_correction;
            home_seen = true;
            break;
        }
    }
    return home_seen;

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
            std::cout << "OBSTACLE CONTROLLER: ShouldInterupt() -> reset" << std::endl;
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

void ObstacleController::SetCurrentTimeInMilliSecs(long int time) {
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

/*
 * Traverse an obstacle
 * NOTE: this is base code traversal methods
 */
void ObstacleController::traversal() {
    result.type = precisionDriving;
    result.pd.setPointVel = 0.0;
    result.pd.cmdVel = 0.0;
    result.pd.setPointYaw = 0;
    switch (this->detection_declaration) {
        case NO_OBSTACLE:
            result.type = waypoint;
            obstacleAvoided = true;
            phys = false;
            obstacleDetected = false;
            can_set_waypoint = true;
            break;
        case OBS_LEFT:
        case OBS_LEFT_CENTER:
        case OBS_CENTER:
            result.pd.cmdAngular = -K_angular;
            break;
        case OBS_RIGHT_CENTER:
        case OBS_RIGHT:
            result.pd.cmdAngular = K_angular;
            break;
        default:
        std::cout << "ERROR: OBSTACLE CONTROLLER: traversal() hit default" << std::endl;
            break;
    }
}

/**
 * Generates a random reflection angle that is bounded by how the rover encounters the obstacle.
 * @param bounds : the lower and upper bounds of the generate angle
 */
void ObstacleController::reflect(std::vector<double> bounds) {
    /*
     * Generate a random rotation angle based on how it encountered the obstacle
     */
    if (this->reflection.should_start) {
        std::cout << "Start Reflect" << std::endl;
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> distribution(bounds.at(0), bounds.at(1));
        double random_angle = distribution(gen);
        this->reflection.desired_heading = angles::normalize_angle(random_angle + currentLocation.theta);
        // Create a reference to guage how far rover has turned
        this->reflection.reflect_angle = fabs(angles::shortest_angular_distance(currentLocation.theta, this->reflection.desired_heading));
        this->reflection.should_start = false;
    }
    /*
     * If the rover has rotated to the desired reflection angle, EXIT
     */
    else if (this->reflection.reflect_angle <= EXIT_ROTATE) {
        std::cout << "END REFLECT" << std::endl;
        this->reflection.should_end = true;

    }
    /*
     * Subtract off the amount rotated
     */
    else {
        // Monitor how far the rover has turned in relation to its desired heading
        std::cout << "Reflecting" << std::endl;
        this->reflection.reflect_angle = fabs(angles::shortest_angular_distance(currentLocation.theta, this->reflection.desired_heading));
    }
}

/**
 * If accepted detections cross 'MIN_THRESH' mark the direction of the obstacle
 * @param accepted_sonar : temporary map of accepted sonar vectors
 * @param delay_type : which monitor are we editing
 */
void ObstacleController::obstacleContactDir(std::map<SONAR, ObstacleAssistant> accepted_sonar, DELAY_TYPE delay_type) {
    LISTENER obstacle = this->monitor_map.at(delay_type);

    // Based on number of sonar sensors
    std::vector<bool> valid_monitors = {false, false, false};
    bool admit = false;
    ObstacleAssistant *left_assist = NULL;
    ObstacleAssistant *center_assist = NULL;
    ObstacleAssistant *right_assist = NULL;
    for (std::map<SONAR, ObstacleAssistant>::iterator it = accepted_sonar.begin(); it != accepted_sonar.end(); ++it) {
        if (it->second.detections.smallest_detection < MIN_THRESH) {
            switch (it->first) {
                case LEFT:
                    left_assist = &it->second;
                    valid_monitors.at(0) = true;
                    break;
                case CENTER:
                    center_assist = &it->second;
                    valid_monitors.at(1) = true;
                    break;
                case RIGHT:
                    right_assist = &it->second;
                    valid_monitors.at(2) = true;
                    break;
                default:
                    std::cout << "OBSTACLE_CONTROLLER: hit default in 'ObstacleController::obstacleContactDir()" << std::endl;
                    break;
            }
        }
        else {
            // Restart respective sonar monitor
            obstacle.sonar_map.at(it->first).monitor->clear();
        }
    }
    /*
     * Check if there are any good detections
     */
    for (auto member : valid_monitors) {
        if (member == true) {
            admit = true;
            break;
        }
    }
    if (admit) {
        /*
         * Sonar overlap is too large to consider the CENTER as anything more than
         * a distinction value for declaring OBS_TYPEs
         */
        if (center_assist && left_assist) {
            obstacle.type = OBS_LEFT_CENTER;
        } else if (center_assist && right_assist) {
            obstacle.type = OBS_RIGHT_CENTER;
        } else if (center_assist) {
            obstacle.type = OBS_CENTER;
        } else if (left_assist) {
            obstacle.type = OBS_LEFT;
        } else if (right_assist) {
            obstacle.type = OBS_RIGHT;
        }
        this->monitor_map.at(delay_type) = obstacle;
    }
    else {
        this->resetDetections(delay_type);
    }
}



/**
 * Iterates over the passed structure and verifies detection are of acceptable range.
 * Note: Acceptable values are messured against 'DELTA' (calculated max dist. the rover can cover in 1 sec.)
 * @param delay_type : which monitor are we editing
 */
void ObstacleController::sonarAnalysis(DELAY_TYPE delay_type) {
    LISTENER obstacle = this->monitor_map.at(delay_type);
    for (auto assistant : obstacle.sonar_map) {
        float prev;
        float curr;
        bool has_begun = false;
        SONAR sonar = assistant.second.sonar;
        obstacle.sonar_map.at(sonar).detections.good_detection = false;
        // Check to make sure the passed monitor has been populated
        if (assistant.second.detections.init_detection) {
            for (auto detection_range : *assistant.second.monitor) {
                if (!has_begun) {
                    curr = detection_range;
                    obstacle.sonar_map.at(sonar).detections.smallest_detection = curr;
                    has_begun = true;
                }
                else {
                    prev = curr;
                    curr = detection_range;
                    double diff = std::fabs(prev - curr);
                    if (diff < DELTA) {
                        obstacle.sonar_map.at(sonar).detections.good_detection = true;
                        // obstacle.sonar_map.at(sonar).detections.smallest_detection = curr;
                    }
                    else {
                        obstacle.sonar_map.at(sonar).detections.good_detection = false;
                        obstacle.sonar_map.at(sonar).detections.smallest_detection = DEFAULT_RANGE;
                        obstacle.sonar_map.at(sonar).monitor->clear();
                        break;
                    }
                }
            }
        }
        this->monitor_map.at(delay_type) = obstacle;
    }
}

/**
 * Adds passed values of sonar detections to respective structures, then checks
 * to make sure structure doesn't exceed specified size 'VECTOR_MAX'
 * @param delay_type : which monitor
 * @param range : the distance of the respective detection
 * @param sonar : which sensor are we referencing
 */
void ObstacleController::sonarMonitor(DELAY_TYPE delay_type, float range, SONAR sonar) {
    LISTENER obstacle = this->monitor_map.at(delay_type);
    if (obstacle.allowed) {
        obstacle.sonar_map.at(sonar).monitor->push_back(range);
        if (obstacle.sonar_map.at(sonar).monitor->size() >= VECTOR_MAX) {
            obstacle.sonar_map.at(sonar).detections.init_detection = true;
        }
        else {
            obstacle.sonar_map.at(sonar).detections.init_detection = false;
        }
        this->monitor_map.at(delay_type) = obstacle; // Assign new values
    }
}

/**
 * Reset the ObstacleAssistant structure when no good detection are made
 * @param delay_type : which 'assistant' is being passed
 */
void ObstacleController::resetObstacle(DELAY_TYPE delay_type) {
    this->monitor_map.at(delay_type).type = NO_OBSTACLE;
    this->monitor_map.at(delay_type).sonar_map = {
            {LEFT, ObstacleAssistant(LEFT)},
            {CENTER, ObstacleAssistant(CENTER)},
            {RIGHT, ObstacleAssistant(RIGHT)}
    };
}

/**
 * Each iteration through should have to recalc detection.
 * Final detection should be checked if it is allowed to end.
 * @param delay_type : which monitor
 */
void ObstacleController::resetDetections(DELAY_TYPE delay_type) {
    this->monitor_map.at(delay_type).type = NO_OBSTACLE;
    // Only reset main detection after previous reflection has finished
    if (this->reflection.should_start && this->detection_declaration != HOME) {
        this->detection_declaration = NO_OBSTACLE;
        obstacleAvoided = true;
    }
}



