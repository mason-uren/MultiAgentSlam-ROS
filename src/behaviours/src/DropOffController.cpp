#include "DropOffController.h"
#include "Utilities.h"

DropOffController::DropOffController() {

    reachedCollectionPoint = false;

    result.type = behavior;
    result.behaviourType = wait;
    result.wristAngle = 0.7;
    result.reset = false;
    interrupt = false;

    circularCenterSearching = false;
    spinner = 0;
    centerApproach = false;
    seenEnoughCenterTags = false;
    prevCount = 0;

    countLeft = 0;
    countRight = 0;
    tagCount = 0; //total home tag count

    isPrecisionDriving = false;
    startWaypoint = false;
    timerTimeElapsed = -1;

    //new globals
    isAligned = false;

}

DropOffController::~DropOffController() {

}

Result DropOffController::DoWork() {


    //cout << "8 I am currently in DropOff mode" << endl;

    logicMessage(current_time, ClassName, __func__);

    //Starts the timer
    if (timerTimeElapsed > -1) {
        long int elapsed = current_time - returnTimer;
        timerTimeElapsed = elapsed / 1e3; // Convert from milliseconds to seconds
    }

    //if we are in the routine for exiting the circle once we have dropped a block off and resetting all our flags
    //to restart our search.
    if (reachedCollectionPoint) {
        //cout << "2 I am at home" << endl;

        dropOffMessage(ClassName, "reached collection point");

        // Check if we have backed far enough away from the dropoff zone
        bool clearOfCircle = timerTimeElapsed > minimumBackupThreshold
                             && closestTagDistance > centerClearedDistanceThreshold;

        if (clearOfCircle) {
            if (finalInterrupt) {
                result.type = behavior;
                result.behaviourType = nextProcess;
                result.reset = true;
                string message = "Exiting DropOff";
                logMessage(current_time, "DROPOFF", message);
                return result;
            } else {
                logMessage(current_time, "DROPOFF", "Clear of circle, entering final interrupt");
                finalInterrupt = true;
                cout << "1" << endl;
            }
        } else if (timerTimeElapsed >= 0.1) {
            isPrecisionDriving = true;
            result.type = precisionDriving;

            DropCube();

            //backing out of home
            result.pd.cmdVel = -0.3;
            result.pd.cmdAngularError = 0.0;

            string message = "Resource released and exiting home";
            logMessage(current_time, "DROPOFF", message);
        }

        return result;
    }

    double distanceToCenter = Utilities::distance_between_points(centerLocation, currentLocation);

    //check to see if we are driving to the center location or if we need to drive in a circle and look.
    if (distanceToCenter > collectionPointVisualDistance && !circularCenterSearching && (tagCount == 0)) {

        WaypointNav();

        return result;

    } else if (timerTimeElapsed >= 2 && tagCount == 0 || !isAligned) //Believes it is home but is not
    {
        SearchForHome(); //currently spin search
    }

    bool centerSeen = tagCount > 0;

    //reset lastCenterTagThresholdTime timout timer to current time
//    if ((!centerApproach && !seenEnoughCenterTags) || (tagCount > 0 && !seenEnoughCenterTags)) {
//        lastCenterTagThresholdTime = current_time;
//        dropOffMessage("setting last time saw tag", "// no tags seen");
//    }

    if(isAligned)
    {
        DeliverCube();
    }

    else if(centerSeen)
    {

        if (first_center && isPrecisionDriving) {
            first_center = false;
            result.type = behavior;
            result.reset = false;
            result.behaviourType = nextProcess;
            dropOffMessage("Mystery action", "type: behavior, behaviourType: next, reset: false");
            return result;
        }

        result.type = precisionDriving;

        if(!isAligned)
        {
            isAligned = Align();
        }
    }

//    if ((tagCount > 0 || seenEnoughCenterTags || prevCount > 0) && !isAligned) //if we have a target and the center is located drive towards it.
//    {
//        string message = "Attempting DropOff";
//        logMessage(current_time, "DROPOFF", message);
//
//        //cout << "9 I have located the center" << endl;
//        //centerSeen = true;
//
//        if (first_center && isPrecisionDriving) {
//            first_center = false;
//            result.type = behavior;
//            result.reset = false;
//            result.behaviourType = nextProcess;
//            dropOffMessage("Mystery action", "type: behavior, behaviourType: next, reset: false");
//            return result;
//        }
//        isPrecisionDriving = true;
//        dropOffMessage("maybe lining up", "precision driving true");
//
//        result.type = precisionDriving;
//
//        if(!isAligned)
//        {
//            isAligned = Align();
//            dropOffMessage("Aligning", "----------");
//        }
//        else
//        {
//            result.pd.cmdVel = 0.12;
//            result.pd.cmdAngularError = 0.0;
//            dropOffMessage("Aligned complete", "-----Driving Forward");
//        }
//
//        //must see greater than this many tags before assuming we are driving into the center and not along an edge.
//        if (tagCount > centerTagThreshold) {
//            seenEnoughCenterTags = true; //we have driven far enough forward to be in and aligned with the circle.
//            lastCenterTagThresholdTime = current_time;
//        }
//        if (tagCount > 0) //reset gaurd to prevent drop offs due to loosing tracking on tags for a frame or 2.
//        {
//            lastCenterTagThresholdTime = current_time;
//        }
//        //time since we dropped below countGuard tags
//        long int elapsed = current_time - lastCenterTagThresholdTime;
//        float timeSinceSeeingEnoughCenterTags = elapsed / 1e3; // Convert from milliseconds to seconds
//
//        //we have driven far enough forward to have passed over the circle.
//        if (tagCount < 1 && seenEnoughCenterTags && timeSinceSeeingEnoughCenterTags > dropDelay) {
//            centerSeen = false;
//        }
//        centerApproach = true;
//        prevCount = tagCount;
//        tagCount = 0;
//        countLeft = 0;
//        countRight = 0;
//    }

        //was on approach to center and did not seenEnoughCenterTags
        //for lostCenterCutoff seconds so reset.
//    else if (centerApproach) {
//        dropOffMessage("Center approach thing", "----------");
//
//        long int elapsed = current_time - lastCenterTagThresholdTime;
//        float timeSinceSeeingEnoughCenterTags = elapsed / 1e3; // Convert from milliseconds to seconds
//        if (timeSinceSeeingEnoughCenterTags > lostCenterCutoff) {
//
//            dropOffMessage("timeSinceSeeingEnoughCenterTags > lostCenterCutoff", "----------");
//            //cout << "4 We have lost the center, drop off attempt abandoned" << endl;
//            //go back to drive to center base location instead of drop off attempt
//            reachedCollectionPoint = false;
//            seenEnoughCenterTags = false;
//            centerApproach = false;
//
//            result.type = waypoint;
//            result.waypoints.push_back(this->centerLocation);
//            if (isPrecisionDriving) {
//                result.type = behavior;
//                result.behaviourType = prevProcess;
//                result.reset = false;
//                dropOffMessage("timeSinceSeeingEnoughCenterTags > lostCenterCutoff", "turning off precision driving");
//            }
//            isPrecisionDriving = false;
//            interrupt = false;
//            precisionInterrupt = false;
//
//            string message = "Abandoning DropOff. Lost sight of home.";
//            logMessage(current_time, "DROPOFF", message);
//        } else {
//            result.pd.cmdVel = searchVelocity;
//            result.pd.cmdAngularError = 0.0;
//            dropOffMessage("center approach", "driving forward");
//        }
//
//        return result;
//
//    }

//    if(!centerSeen && seenEnoughCenterTags) {
//        reachedCollectionPoint = true;
//        centerApproach = false;
//        returnTimer = current_time;
//    }

    return result;
}

//Rotates bot to better align with home tags
bool DropOffController::Align()
{

    //still need to add condition where yaw is +-1.5
    if(tagYaw >= 0.08)// turn right
    {
        result.pd.cmdAngular = 0.7;
        result.pd.cmdAngularError = -0.12;
        dropOffMessage(ClassName, "Align right");
    }
    else if(tagYaw <= -0.08)//turn left
    {
        result.pd.cmdAngular = 0.7;
        result.pd.cmdAngularError = 0.12;
        dropOffMessage(ClassName, "Align left");
    }

    if((tagYaw > -0.08 && tagYaw < 0.08))
    {
        result.pd.cmdAngular = 0.0;
        return true;
    }

    return false;
}


//drives forward until no tags seen
//sets reachedCollectionPoint which triggers next action
void DropOffController::DeliverCube()
{
    if(tagCount > 0) //while tags are still seen
    {
        dropOffMessage(ClassName, __func__);
        result.pd.cmdVel = 0.10;
        result.pd.cmdAngularError = 0.0;

    }
    else
    {
        dropOffMessage(ClassName, "deliver should stop");
        result.pd.cmdVel = 0.0;
        result.pd.cmdAngularError = 0.0;
        reachedCollectionPoint = true; //will trigger releasing cube and backing out
        returnTimer = current_time;
    }
}

void DropOffController::DropCube()
{
    dropOffMessage(ClassName, __func__);
    result.fingerAngle = M_PI_2; //open fingers
    result.wristAngle = 0; //raise wrist
}

void DropOffController::WaypointNav()
{
    dropOffMessage(ClassName, __func__);
    result.type = waypoint;
    result.waypoints.clear();
    result.waypoints.push_back(this->centerLocation);
    startWaypoint = false;
    isPrecisionDriving = false;

    timerTimeElapsed = 0;

    string message = "Starting DropOff, setting waypoint to center location";
    logMessage(current_time, "DROPOFF", message);
}

void DropOffController::SearchForHome()
{
    dropOffMessage(ClassName, __func__);
    Point nextSpinPoint;

    //sets a goal that is 60cm from the centerLocation and spinner
    //radians counterclockwise from being purly along the x-axis.
    nextSpinPoint.x = centerLocation.x + (initialSpinSize + spinSizeIncrease) * cos(spinner);
    nextSpinPoint.y = centerLocation.y + (initialSpinSize + spinSizeIncrease) * sin(spinner);
    nextSpinPoint.theta = Utilities::angle_between_points(nextSpinPoint,currentLocation);

    result.type = waypoint;
    result.waypoints.clear();
    result.waypoints.push_back(nextSpinPoint);

    spinner += 45 * (M_PI / 180); //add 45 degrees in radians to spinner.
    if (spinner > 2 * M_PI) {
        spinner -= 2 * M_PI;
    }
    spinSizeIncrease += spinSizeIncrement / 8;
    circularCenterSearching = true;
    //safety flag to prevent us trying to drive back to the
    //center since we have a block with us and the above point is
    //greater than collectionPointVisualDistance from the center.

    returnTimer = current_time;
    timerTimeElapsed = 0;
}

void DropOffController::Reset() {
    dropOffMessage(ClassName, __func__);
    result.type = behavior;
    result.behaviourType = wait;
    result.pd.cmdVel = 0;
    result.pd.cmdAngularError = 0;
    result.fingerAngle = -1;
    result.wristAngle = 0.7;
    result.reset = false;
    result.waypoints.clear();
    spinner = 0;
    spinSizeIncrease = 0;
    prevCount = 0;
    timerTimeElapsed = -1;

    countLeft = 0;
    countRight = 0;
    closestTagDistance = 0;
    tagCount = 0;

    //reset flags
    reachedCollectionPoint = false;
    seenEnoughCenterTags = false;
    circularCenterSearching = false;
    isPrecisionDriving = false;
    finalInterrupt = false;
    precisionInterrupt = false;
    targetHeld = false;
    startWaypoint = false;
    first_center = true;

    isAligned = false;
        //cout << "6 Reset has occurred" << endl;

}

void DropOffController::SetTargetData(vector<Tag> tags) {
    countRight = 0;
    countLeft = 0;
    tagCount = 0;
    double roll, pitch;
    double tagDistance;

    int closestIdx = getClosestCenterTagIdx(tags);

    if (closestIdx > -1) {
        // Found a tag
        closestTagDistance = tagDistanceFromCamera(tags[closestIdx]);
    }

    tf::Quaternion tagOrien(tags[closestIdx].getOrientationX(), tags[closestIdx].getOrientationY(), tags[closestIdx].getOrientationZ(), tags[closestIdx].getOrientationW());
    tf::Matrix3x3 rotMartrix(tagOrien);
    rotMartrix.getRPY(roll, pitch, tagYaw);


    // this loop is to get the number of center tags
    for (auto &tag : tags) {
        if (tag.getID() == 256) {
            // if a target is detected and we are looking for center tags
            if (targetHeld && !reachedCollectionPoint) {
                // checks if tag is on the right or left side of the image
                if (tag.getPositionX() + cameraOffsetCorrection > 0) {
                    countRight++;
                } else {
                    countLeft++;
                }
            }
        }
    }
    tagCount = countLeft + countRight; //total home tags
    tagMessage(tags);
}

void DropOffController::ProcessData() {
    if ((countLeft + countRight) > 0) {
        isPrecisionDriving = true;
    } else {
        startWaypoint = true;
    }
}

bool DropOffController::ShouldInterrupt() {
    ProcessData();
    if (startWaypoint && !interrupt) {
        interrupt = true;
        precisionInterrupt = false;
        return true;
    } else if (isPrecisionDriving && !precisionInterrupt) {
        precisionInterrupt = true;
        return true;
    }
    if (finalInterrupt) {
        return true;
    }
}

bool DropOffController::HasWork() {


    if (timerTimeElapsed > -1) {
        long int elapsed = current_time - returnTimer;
        timerTimeElapsed = elapsed / 1e3; // Convert from milliseconds to seconds
    }

    if (circularCenterSearching && timerTimeElapsed < 2 && !isPrecisionDriving) {

        return false;
    }

    return ((startWaypoint || isPrecisionDriving));
}

bool DropOffController::IsChangingMode() {
    return isPrecisionDriving;
}

void DropOffController::SetCenterLocation(Point center) {
    centerLocation = center;
}

void DropOffController::SetCurrentLocation(Point current) {
    currentLocation = current;
}

void DropOffController::SetTargetPickedUp() {
    targetHeld = true;
}

void DropOffController::SetBlockBlockingUltrasound(bool blockBlock) {
    targetHeld = targetHeld || blockBlock;
}

void DropOffController::SetCurrentTimeInMilliSecs(long int time) {
    current_time = time;
}

int DropOffController::getClosestCenterTagIdx(const vector<Tag> &tags) {
    int idx = -1;
    double closest = std::numeric_limits<double>::max();

    for (int i = 0; i < tags.size(); i++) {
        if (tags[i].getID() == 256) {
            double distance = tagDistanceFromCamera(tags[i]);

            if (distance < closest) {
                idx = i;
                closest = distance;
            }
        }
    }

    return idx;
}

double DropOffController::tagDistanceFromCamera(const Tag &tag) {
    return hypot(hypot(tag.getPositionX(), tag.getPositionY()), tag.getPositionZ());
}

Point closestAnchor(Point current) {
    int x = current.x;
    int y = current.y;
    Point nearestAnchor;
    if (x > 0 && y > 0) //if true in quadrant I
    {
        if (y > x)//upper triangle of quadrant I
        {
            nearestAnchor.x = 0;
            nearestAnchor.y = 1;//rover goes to (0,1)
        } else {
            nearestAnchor.x = 1;
            nearestAnchor.y = 0;//rover goes to (1, 0)
        }

    } else if (x > 0 && y < 0) //if true in quadrant II
    {
        if (abs(y) > x)//lower triangle of quadrant II
        {
            nearestAnchor.x = 0;
            nearestAnchor.y = -1;//rover goes to (0,-1)
        } else //upper triangle of quadrant II
        {
            nearestAnchor.x = 1;
            nearestAnchor.y = 0;//rover goes to (1, 0)
        }
    } else if (x < 0 && y > 0)//if true in quadrant IV
    {
        if (y > x)//upper triangle of quadrant IV
        {
            nearestAnchor.x = 0;
            nearestAnchor.y = 1;//rover goes to (0,1)
        } else {
            nearestAnchor.x = -1;
            nearestAnchor.y = 0;//rover goes to (-1, 0)
        }

    } else if (x < 0 && y < 0) //if true in quadrant III
    {
        if (abs(y) > x)//upper triangle of quadrant III
        {
            nearestAnchor.x = 0;
            nearestAnchor.y = -1;//rover goes to (0,-1)
        } else {
            nearestAnchor.x = -1;
            nearestAnchor.y = 0;//rover goes to (-1, 0)
        }

    }
    return nearestAnchor;
}
