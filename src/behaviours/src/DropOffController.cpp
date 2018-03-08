#include "DropOffController.h"

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

    isPrecisionDriving = false;
    startWaypoint = false;
    timerTimeElapsed = -1;

}

DropOffController::~DropOffController() {

}

Result DropOffController::DoWork() {


  //cout << "8 I am currently in DropOff mode" << endl;

    logicMessage(current_time, ClassName, __func__);
    int count = countLeft + countRight;

    if (timerTimeElapsed > -1) {
        long int elapsed = current_time - returnTimer;
        timerTimeElapsed = elapsed / 1e3; // Convert from milliseconds to seconds
    }

    //if we are in the routine for exiting the circle once we have dropped a block off and reseting all our flags
    //to resart our search.
    if (reachedCollectionPoint) {
        //cout << "2 I am at home" << endl;
        if (timerTimeElapsed >= 5) {
            if (finalInterrupt) {
                result.type = behavior;
                result.behaviourType = nextProcess;
                string message = "Exiting DropOff";
                logMessage(current_time, "DROPOFF", message);

                cout << "result probably 0 here: " << result.type << endl;
                cout << "backwaypoint before: " << result.waypoints.back().x << result.waypoints.back().y << endl;
                cout << "back waypoint after: " << result.waypoints.back().x << result.waypoints.back().y << endl;

                result.reset = true;
                return result;
            } else {
                finalInterrupt = true;
                cout << "1" << endl;
            }
        } else if (timerTimeElapsed >= 0.1) {
            isPrecisionDriving = true;
            result.type = precisionDriving;

            result.fingerAngle = M_PI_2; //open fingers
            result.wristAngle = 0; //raise wrist

            result.pd.cmdVel = -0.3;
            result.pd.cmdAngularError = 0.0;

            string message = "Resource released and exiting home";
            logMessage(current_time, "DROPOFF", message);
        }

        return result;
    }

    double distanceToCenter = distance_between_points(centerLocation,currentLocation);

    //check to see if we are driving to the center location or if we need to drive in a circle and look.
    if (distanceToCenter > collectionPointVisualDistance && !circularCenterSearching && (count == 0)) {

        result.type = waypoint;
        result.waypoints.clear();
        result.waypoints.push_back(this->centerLocation);
        startWaypoint = false;
        isPrecisionDriving = false;

        timerTimeElapsed = 0;

        string message = "Starting DropOff, setting waypoint to center location";
        logMessage(current_time, "DROPOFF", message);

        return result;

    } else if (timerTimeElapsed >= 2)//spin search for center
    {
        Point nextSpinPoint;

        //sets a goal that is 60cm from the centerLocation and spinner
        //radians counterclockwise from being purly along the x-axis.
        nextSpinPoint.x = centerLocation.x + (initialSpinSize + spinSizeIncrease) * cos(spinner);
        nextSpinPoint.y = centerLocation.y + (initialSpinSize + spinSizeIncrease) * sin(spinner);
        nextSpinPoint.theta = angle_between_points(nextSpinPoint,currentLocation);

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

    bool left = (countLeft > 0);
    bool right = (countRight > 0);
    bool centerSeen = (right || left);

    //reset lastCenterTagThresholdTime timout timer to current time
    if ((!centerApproach && !seenEnoughCenterTags) || (count > 0 && !seenEnoughCenterTags)) {
        lastCenterTagThresholdTime = current_time;
    }

    if (count > 0 || seenEnoughCenterTags ||
        prevCount > 0) //if we have a target and the center is located drive towards it.
    {
        string message = "Attempting DropOff";
        logMessage(current_time, "DROPOFF", message);

        //cout << "9 I have located the center" << endl;
        centerSeen = true;

        if (first_center && isPrecisionDriving) {
            first_center = false;
            result.type = behavior;
            result.reset = false;
            result.behaviourType = nextProcess;
            return result;
        }
        isPrecisionDriving = true;

        if (seenEnoughCenterTags) //if we have seen enough tags
        {
            if ((countLeft - 5) > countRight) //and there are too many on the left
            {
                right = false; //then we say none on the right to cause us to turn right
            } else if ((countRight - 5) > countLeft) {
                left = false; //or left in this case
            }
        }

        float turnDirection = 1;
        //reverse tag rejection when we have seen enough tags that we are on a
        //trajectory in to the square we dont want to follow an edge.
        if (seenEnoughCenterTags) turnDirection = -3;

        result.type = precisionDriving;

        //otherwise turn till tags on both sides of image then drive straight
        if (left && right) {
            result.pd.cmdVel = searchVelocity;
            result.pd.cmdAngularError = 0.0;
        } else if (right) {
            result.pd.cmdVel = -0.1 * turnDirection;
            result.pd.cmdAngularError = -centeringTurnRate * turnDirection;
        } else if (left) {
            result.pd.cmdVel = -0.1 * turnDirection;
            result.pd.cmdAngularError = centeringTurnRate * turnDirection;
        } else {
            result.pd.cmdVel = searchVelocity;
            result.pd.cmdAngularError = 0.0;
        }

        //must see greater than this many tags before assuming we are driving into the center and not along an edge.
        if (count > centerTagThreshold) {
            seenEnoughCenterTags = true; //we have driven far enough forward to be in and aligned with the circle.
            lastCenterTagThresholdTime = current_time;
        }
        if (count > 0) //reset gaurd to prevent drop offs due to loosing tracking on tags for a frame or 2.
        {
            lastCenterTagThresholdTime = current_time;
        }
        //time since we dropped below countGuard tags
        long int elapsed = current_time - lastCenterTagThresholdTime;
        float timeSinceSeeingEnoughCenterTags = elapsed / 1e3; // Convert from milliseconds to seconds

        //we have driven far enough forward to have passed over the circle.
        if (count < 1 && seenEnoughCenterTags && timeSinceSeeingEnoughCenterTags > dropDelay) {
            centerSeen = false;
        }
        centerApproach = true;
        prevCount = count;
        count = 0;
        countLeft = 0;
        countRight = 0;
    }

        //was on approach to center and did not seenEnoughCenterTags
        //for lostCenterCutoff seconds so reset.
    else if (centerApproach) {

        long int elapsed = current_time - lastCenterTagThresholdTime;
        float timeSinceSeeingEnoughCenterTags = elapsed / 1e3; // Convert from milliseconds to seconds
        if (timeSinceSeeingEnoughCenterTags > lostCenterCutoff) {
            //cout << "4 We have lost the center, drop off attempt abandoned" << endl;
            //go back to drive to center base location instead of drop off attempt
            reachedCollectionPoint = false;
            seenEnoughCenterTags = false;
            centerApproach = false;

            result.type = waypoint;
            result.waypoints.push_back(this->centerLocation);
            if (isPrecisionDriving) {
                result.type = behavior;
                result.behaviourType = prevProcess;
                result.reset = false;
            }
            isPrecisionDriving = false;
            interrupt = false;
            precisionInterrupt = false;

            string message = "Abandoning DropOff. Lost sight of home.";
            logMessage(current_time, "DROPOFF", message);
        } else {
            result.pd.cmdVel = searchVelocity;
            result.pd.cmdAngularError = 0.0;
        }

        return result;

    }

    if (!centerSeen && seenEnoughCenterTags) {
        reachedCollectionPoint = true;
        centerApproach = false;
        returnTimer = current_time;
    }

    return result;
}

void DropOffController::Reset() {
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
        //cout << "6 Reset has occurred" << endl;

}

void DropOffController::SetTargetData(vector <Tag> tags) {
    countRight = 0;
    countLeft = 0;

    if (targetHeld) {
        // if a target is detected and we are looking for center tags
        if (tags.size() > 0 && !reachedCollectionPoint) {

            // this loop is to get the number of center tags
            for (int i = 0; i < tags.size(); i++) {
                if (tags[i].getID() == 256) {

                    // checks if tag is on the right or left side of the image
                    if (tags[i].getPositionX() + cameraOffsetCorrection > 0) {
                        countRight++;

                    } else {
                        countLeft++;
                    }
                }
            }
        }
    }

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

Point closestAnchor(Point current){
    int x = current.x;
    int y = current.y;
    Point nearestAnchor;
    if(x > 0 && y > 0) //if true in quadrant I
    {
        if(y > x)//upper triangle of quadrant I
        {
            nearestAnchor.x = 0;
            nearestAnchor.y = 1;//rover goes to (0,1)
        }
        else
        {
            nearestAnchor.x = 1;
            nearestAnchor.y = 0;//rover goes to (1, 0)
        }

    }
    else if(x > 0 && y < 0) //if true in quadrant II
    {
        if(abs(y) > x)//lower triangle of quadrant II
        {
            nearestAnchor.x = 0;
            nearestAnchor.y = -1;//rover goes to (0,-1)
        }
        else //upper triangle of quadrant II
        {
            nearestAnchor.x = 1;
            nearestAnchor.y = 0;//rover goes to (1, 0)
        }
    }
    else if(x < 0 && y > 0)//if true in quadrant IV
    {
        if(y > x)//upper triangle of quadrant IV
        {
            nearestAnchor.x = 0;
            nearestAnchor.y = 1;//rover goes to (0,1)
        }
        else
        {
            nearestAnchor.x = -1;
            nearestAnchor.y = 0;//rover goes to (-1, 0)
        }

    }
    else if(x < 0 && y < 0) //if true in quadrant III
    {
        if(abs(y) > x)//upper triangle of quadrant III
        {
            nearestAnchor.x = 0;
            nearestAnchor.y = -1;//rover goes to (0,-1)
        }
        else
        {
            nearestAnchor.x = -1;
            nearestAnchor.y = 0;//rover goes to (-1, 0)
        }

    }
    return nearestAnchor;
}
