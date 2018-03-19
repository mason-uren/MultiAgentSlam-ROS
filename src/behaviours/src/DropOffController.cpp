// This file deals with the rover's ability to drop off cubes to the center collection disk
// There are only two forms of driving: precision driving and waypoints
// Precision Driving == any controller (drive, pickup, dropoff, obstacle)
// continously feeding data into the feedback loop needed for drive controls
// has more precise control over rover's movements, more accurate of less than 1cm

// Waypoint Driving == drive controller feeding one data point (waypoint coordinates)
// with an accuracy of at least 15cm

#include "DropOffController.h"

// Constructor to set initial values
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
    countCenter = 0;
    tagCount = 0; //total home tag count
    alignAngleSumLeft = 0;
    alignAngleSumRight = 0;

    isPrecisionDriving = false;
    startWaypoint = false;
    timerTimeElapsed = 0;
    alignTimer = -1;
    realignTimer = -1;
    deliverTimer = -1;

    //new globals
    isAligned = false;
    firstAlign = false;
    firstReAlign = false;
    edgeCase = false;
    alternateDeliver = false;
    homeFound = false;
    startDeliverTimer = false;
    noLeft = false;
    noRight = false;
    altAlignCorner = false;
    altAlignEdge = false;

    this->controller = DROP_OFF;

}

DropOffController::~DropOffController() {

}

Result DropOffController::DoWork() {


    //cout << "8 I am currently in DropOff mode" << endl;

    logicMessage(current_time, ClassName, __func__);

    bool centerSeen = tagCount > 0;
    double distanceToCenter = distance_between_points(centerLocation, currentLocation);

    //Starts the timer
    if (timerTimeElapsed > -1) {
        long int elapsed = current_time - returnTimer;
        timerTimeElapsed = elapsed / 1e3; // Convert from milliseconds to seconds
    }

    //if we are in the routine for exiting the circle once we have dropped a block off and resetting all our flags
    //to restart our search.
    if (reachedCollectionPoint) {

        // Check if we have backed far enough away from the dropoff zone
        bool clearOfCircle = timerTimeElapsed > minimumBackupThreshold && tagCount == 0;
                             //&& closestTagDistance > centerClearedDistanceThreshold;

        cout << "clear of circle values: " <<
                timerTimeElapsed << " " <<
                tagCount << " " <<
                closestTagDistance << endl;


        dropOffMessage(ClassName, "reached collection point");

        if (clearOfCircle) {
            cout << "is clear of circle " << endl;
            dropOffMessage(ClassName, "clear of circle");
            if (finalInterrupt) {
                result.type = behavior;
                result.behaviourType = nextProcess;
                result.reset = true;
                cout << "final interrupt *******************" << endl;
                logMessage(current_time, ClassName, "Exiting DropOff");
                return result;
            } else {
                cout << "NOT final interrupt" << endl;
                logMessage(current_time, "DROPOFF", "Clear of circle, entering final interrupt");
                finalInterrupt = true;
            }
        } else if (timerTimeElapsed >= 0.1)
        {
            cout << "timer time elapsed about to start backup" << endl;
            dropOffMessage(ClassName, "about to call Backup()");
            BackUp();
        }

        return result;
    }

    //check to see if we are driving to the center location or if we need to drive in a circle and look.
    if (distanceToCenter > collectionPointVisualDistance && !circularCenterSearching && (tagCount == 0) && !homeFound) {

        WaypointNav();

        return result;

    }
    else if (timerTimeElapsed >= 2 && tagCount == 0 && !homeFound) //Believes it is home but is not
    {
        cout << " ET PHONE HOME " << endl;
        SearchForHome(); //currently spin search
    }

//    if(alternateDeliver)
//    {
//        AltDeliver();
//    }
//    else
    if(isAligned)
    {
        DeliverCube();
        result.enable_reset_center_location = false;
    }
    else if(centerSeen)
    {

        if (first_center && isPrecisionDriving) {
            first_center = false;
            result.type = behavior;
            result.reset = false;
            result.behaviourType = nextProcess;
            cout << "STEP 1: Weird mystery code" << endl;
            return result;
        }

        result.type = precisionDriving;
        homeFound = true;

        if (!isAligned)
        {
            Align();
        }
    }

    return result;
}

//Rotates bot to better align with home tags
void DropOffController::Align()
{
    dropOffMessage(ClassName, __func__);

    float blockDistanceFromCamera;

    float blockYawError = 0.0;
    // using a^2 + b^2 = c^2 to find the distance to the block
    // 0.195 is the height of the camera lens above the ground in cm.
    //
    // a is the linear distance from the robot to the block, c is the
    // distance from the camera lens, and b is the height of the
    // camera above the ground.
    blockDistanceFromCamera = hypot(hypot(average_center_tag.getPositionX(), average_center_tag.getPositionY()), average_center_tag.getPositionZ());


    if ((blockDistanceFromCamera * blockDistanceFromCamera - 0.195 * 0.195) > 0) {
        blockDistance = sqrt(blockDistanceFromCamera * blockDistanceFromCamera - 0.195 * 0.195);
    } else {
        float epsilon = 0.00001; // A small non-zero positive number
        blockDistance = epsilon;
    }

    blockYawError = atan((average_center_tag.getPositionX() + cameraOffsetCorrection) / blockDistance) * 1.05; //angle to block from bottom center of chassis on the horizontal.
    // cout << average_center_tag.getPositionX() << endl;
    if(abs(average_center_tag.getPositionX()) < 0.02)
    {
        cout << "STEP2: Stopping Rotation." << endl;
        startDeliveryTime = current_time;
        result.type = precisionDriving;
        result.pd.cmdVel = 0.00;
        result.pd.cmdAngular = 0.0;
        result.pd.cmdAngularError = 0.0;
        // result.pd.cmdAngular = -200*average_center_tag.getPositionX();
        result.pd.setPointVel = 0.0;
        result.pd.setPointYaw = 0.0;
        prevYaw = tagYaw;
        isAligned = true;
        result.enable_reset_center_location = true;
    }
    result.type = precisionDriving;
    result.pd.cmdVel = 0.10;
    result.pd.cmdAngular = 0.0;
    result.pd.cmdAngularError = -2.0*(cameraOffsetCorrection + average_center_tag.getPositionX());
    // result.pd.cmdAngular = -200*average_center_tag.getPositionX();
    result.pd.setPointVel = 0.0;
    result.pd.setPointYaw = 0.0;



//    if (tagYaw >= 0.07)// turn right
//    {
//        result.pd.cmdAngular = 0.7;
//        result.pd.cmdAngularError = -0.12;
//        alignAngleSumRight += -0.12;
//        dropOffMessage(ClassName, "Align right");
//
//        if (abs(alignAngleSumRight) >= 0.48 && countCenter == 0) {
//            dropOffMessage(ClassName, "Align no right tags");
//            noRight = true;
//        } else {
//            noRight = false;
//        }
//    }
//    else if (tagYaw <= -0.07)//turn left
//    {
//        result.pd.cmdAngular = 0.7;
//        result.pd.cmdAngularError = 0.12;
//        alignAngleSumLeft += 0.12;
//        dropOffMessage(ClassName, "Align left");
//
//        if (alignAngleSumLeft >= 0.35 && countCenter == 0) {
//            dropOffMessage(ClassName, "Align no left tags");
//            noLeft = true;
//        } else {
//            noLeft = false;
//        }
//    }
//
//    if ((tagYaw > -0.06 && tagYaw < 0.08)) {
//        result.pd.cmdAngular = 0.0;
//        prevYaw = tagYaw;
//        isAligned = true;
//        alignAngleSumLeft = 0;
//        alignAngleSumRight = 0;
//    }
//    else if ((abs(alignAngleSumRight) > 3 || alignAngleSumLeft > 3 ||(alignAngleSumLeft + abs(alignAngleSumRight)) > 5)) {
//        dropOffMessage(ClassName, "Aligned failed, time for something else");
//        if(tagYaw > -0.2 && tagYaw > 0.2)
//        {
//            altAlignCorner = true;
//        }
//        alignAngleSumLeft = 0;
//        alignAngleSumRight = 0;
//    }
}

//drives forward until no tags seen
//sets reachedCollectionPoint which triggers next action
void DropOffController::DeliverCube()
{
    dropOffMessage(ClassName, __func__);

    deliverTimer = (current_time - startDeliveryTime) / 1e3;
    if(countCenter > 0) //while tags are still seen
    {
        //cout << deliveryTimer << endl;
        //cout << ((blockDistance+0.3)/0.2) << endl;
        if(deliverTimer < ((blockDistance+0.3)/0.2)){
            dropOffMessage(ClassName, __func__);
            result.pd.cmdVel = 0.2;
            result.pd.cmdAngularError = 0.0;
        }
//        if((abs(centerYaw) - abs(prevYaw) > 0.75 || abs(centerYaw) - abs(prevYaw) < -0.75))
//        {
//            dropOffMessage(ClassName, "Deliver - seen funky tags. sending to altDeliver");
//            cout << "STEP Plan B: Alternate deliver" << endl;
//            alternateDeliver = true;
//        }
        else
        {
            result.pd.cmdVel = 0.0;
            result.pd.cmdAngularError = 0.0;
            reachedCollectionPoint = true; //will trigger releasing cube and backing out
            returnTimer = current_time;
            cout << "STEP3: Delivering Cube and moved desired distance." << endl;
        }
    }
    else
    {
        dropOffMessage(ClassName, "deliver should stop");
        result.pd.cmdVel = 0.0;
        result.pd.cmdAngularError = 0.0;
        reachedCollectionPoint = true; //will trigger releasing cube and backing out
        returnTimer = current_time;
        cout << "STEP3: Delivering Cube and not longer see home tags." << endl;
    }
}

void DropOffController::AltDeliver()
{
    dropOffMessage(ClassName, __func__);
    if((centerYaw >= 0 || countRight <= 1) && tagCount > 0) //rotate right
    {
        result.pd.cmdVel = 0.1;
        result.pd.cmdAngular = 0.7;
        result.pd.cmdAngularError = -0.12;
    }
    else if((centerYaw < 0 || countLeft <= 1) && tagCount > 0) //rotate left
    {
        result.pd.cmdVel = 0.1;
        result.pd.cmdAngular = 0.7;
        result.pd.cmdAngularError = 0.12;
    }
    else
    {
        result.pd.cmdAngular = 0.0;
        result.pd.cmdAngularError = 0.0;
        reachedCollectionPoint = true;
        returnTimer = current_time;
    }
}

void DropOffController::DropCube()
{
    dropOffMessage(ClassName, __func__);
    result.fingerAngle = M_PI_2; //open fingers
    result.wristAngle = 0; //raise wrist
}

void DropOffController::BackUp()
{
    logicMessage(current_time, ClassName, __func__);
    logMessage(current_time, ClassName, "Resource released and exiting home");
    dropOffMessage(ClassName, __func__);

    isPrecisionDriving = true;
    result.type = precisionDriving;

    cout << "in backup starting dropCube " << endl;

    DropCube();

    cout << "in backup " << endl;

    //backing out of home
    result.pd.cmdVel = -0.3;
    result.pd.cmdAngularError = 0.0;
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

void DropOffController::Reset() {
    dropOffMessage(ClassName, __func__);
    cout << "STEP4: Dropoff Reset has been called." << endl;
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
    timerTimeElapsed = 0;
    deliverTimer = -1;

    countLeft = 0;
    countRight = 0;
    countCenter = 0;
    closestTagDistance = 0;
    tagCount = 0;

    alignAngleSumLeft = 0;
    alignAngleSumRight = 0;

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
    firstAlign = false;
    firstReAlign = false;
    edgeCase = false;
    alternateDeliver = false;
    homeFound = false;
    startDeliverTimer = false;
    altAlignCorner = false;
    altAlignEdge = false;
}

// Individually calculates and sets the number of tags seen on the right and the left of the rover
void DropOffController::SetTargetData(vector<Tag> tags) {

    countRight = 0;
    countLeft = 0;
    countCenter = 0;
    tagCount = 0;
    double roll, pitch;
    double tagDistance;
    float sum_x = 0.0;
    float sum_y = 0.0;
    float sum_z = 0.0;

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
                if (tag.getPositionX() + cameraOffsetCorrection > 0.02) {
                    countRight++;
                }
                else if (tag.getPositionX() + cameraOffsetCorrection <= 0.02 || tag.getPositionX() + cameraOffsetCorrection >= -0.02) {
                    countCenter++;
                    tf::Quaternion tagOrien(tag.getOrientationX(), tag.getOrientationY(), tag.getOrientationZ(), tag.getOrientationW());
                    tf::Matrix3x3 rotMartrix(tagOrien);
                    rotMartrix.getRPY(roll, pitch, centerYaw);

                } else {
                    countLeft++;
                }
                sum_x = sum_x + tag.getPositionX();
                sum_y = sum_y + tag.getPositionY();
                sum_z = sum_z + tag.getPositionZ();
            }
        }
        average_center_tag.setPositionX(sum_x / (countLeft + countRight + countCenter));
        average_center_tag.setPositionY(sum_y / (countLeft + countRight + countCenter));
        average_center_tag.setPositionZ(sum_z / (countLeft + countRight + countCenter));
    }
    tagCount = countLeft + countCenter + countRight; //total home tags
    tagMessage(tags);
}

// Sets the driving mode (precision or waypoint) depending on the
// number of tags seen on the left and the right side of the rover
void DropOffController::ProcessData() {
    if ((tagCount) > 0) {
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

// Checking function to see if the driving mode (precision or waypoint) has been changed
bool DropOffController::IsChangingMode() {
    return isPrecisionDriving;
}

// Setter function to set the center location (the collection disk)
// Of the Point class (x, y, theta)
void DropOffController::SetCenterLocation(Point center) {
    centerLocation = center;
}

// Setter function to set the current location of the Point class (x, y, theta)
void DropOffController::SetCurrentLocation(Point current) {
    currentLocation = current;
}

// Setter function to set the variable to true if a target (cube) has been picked up
// And that it is currently holding the target (cube)
void DropOffController::SetTargetPickedUp() {
    targetHeld = true;
}

// Setter function to stop the ultrasound from being blocked
// In other words, to block the ultrasound or not
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
