#include "messages.hpp"

using namespace std;

void messageHandler(const std_msgs::String::ConstPtr& message, int* self_idx, std::string* memberNames, const std::string& publishedName, int& swarmSize,
                    std_msgs::Int16& targetCollected, bool* targetsDetected, geometry_msgs::Pose2D* targetPositions, Path* paths, int& stateMachineState,
                    geometry_msgs::Pose2D& goalLocation, geometry_msgs::Pose2D& currentLocation)
{
    string msg = message->data;

    // cout << msg << " " << publishedName << endl;

    size_t type_pos = msg.find_first_of(" ");
    string type = msg.substr(0, type_pos);
    msg = msg.substr(type_pos+1);

    vector<string> msg_parts;

    size_t cur_tok = msg.find_first_of(" ");;
    while(cur_tok != string::npos) { // until end of string
        msg_parts.push_back(msg.substr(0, cur_tok));
        msg = msg.substr(cur_tok + 1);
        cur_tok = msg.find_first_of(" ");
    }

    msg_parts.push_back(msg);

    if(type == "I") {

        identityMessage(msg_parts, self_idx, memberNames, publishedName, swarmSize);

    } else if(type == "D") {

        detectedMessage(msg_parts, targetCollected, targetsDetected, targetPositions, paths[*self_idx], stateMachineState, goalLocation, currentLocation);

    } else if(type == "PA") {

        pointAddedMessage(msg_parts, currentLocation, self_idx, paths);

    } else if(type == "PR") {

        pointRemovedMessage(msg_parts, self_idx, paths);

    }

}

void identityMessage(vector<string> msg_parts, int* self_idx, std::string* memberNames, const std::string& publishedName, int& swarmSize)
{
    if(swarmSize >= 6) {
        return;
    }

    int insert_idx = swarmSize - 1;
    string name = msg_parts[0];

    while(insert_idx >= 0 && name < memberNames[insert_idx]) {
        memberNames[insert_idx + 1] = memberNames[insert_idx];
        insert_idx--;
    }

    memberNames[insert_idx + 1] = name;

    if(name == publishedName) {
        *self_idx = insert_idx + 1;
    }

    if(swarmSize < 6) {
        swarmSize++;
    }
}

void detectedMessage(vector<string> msg_parts, std_msgs::Int16& targetCollected, bool* targetsDetected, geometry_msgs::Pose2D* targetPositions,
                     Path& path, int& stateMachineState, geometry_msgs::Pose2D& goalLocation, geometry_msgs::Pose2D& currentLocation)
{
    stringstream converter;

    std_msgs::Int16 tmp;
    double x, y;
    x = y = 0.0;

    tmp.data = -1; //uninitialized data is the worst to debug

    converter << msg_parts[0];
    converter >> tmp.data;
    converter.str("");
    converter.clear();

    converter << msg_parts[1];
    converter >> x;
    converter.str("");
    converter.clear();

    converter << msg_parts[2];
    converter >> y;
    converter.str("");
    converter.clear();

    targetsDetected[tmp.data] = 1;
    targetPositions[tmp.data].x = x;
    targetPositions[tmp.data].y = y;

    path.Add(currentLocation.x, currentLocation.y, currentLocation.theta, x, y);

    if(targetCollected.data == -1) {
        double theta = atan2(currentLocation.y - y, currentLocation.x - x);

        goalLocation.x = x;
        goalLocation.y = y;
        goalLocation.theta = theta;

        stateMachineState = STATE_MACHINE_TRANSFORM;
    }
}

void pointAddedMessage(vector<string> msg_parts, geometry_msgs::Pose2D& currentLocation, int* self_idx, Path* paths)
{
    stringstream converter;

    int rover_id = -1;
    size_t idx = (size_t)(-1);
    double cx, cy, ct, x, y;
    cx = cy = ct = x = y = 0.0;

    converter << msg_parts[0];
    converter >> rover_id;
    converter.str("");
    converter.clear();

    if(rover_id == *self_idx) {
        return;
    }

    converter << msg_parts[1];
    converter >> idx;
    converter.str("");
    converter.clear();

    converter << msg_parts[2];
    converter >> cx;
    converter.str("");
    converter.clear();

    converter << msg_parts[3];
    converter >> cy;
    converter.str("");
    converter.clear();

    converter << msg_parts[4];
    converter >> ct;
    converter.str("");
    converter.clear();

    converter << msg_parts[5];
    converter >> x;
    converter.str("");
    converter.clear();

    converter << msg_parts[6];
    converter >> y;
    converter.str("");
    converter.clear();

    if(rover_id > 0 && rover_id != *self_idx) {
        paths[rover_id].Insert(idx, cx, cy, ct, x, y);
    }
}

void pointRemovedMessage(vector<string> msg_parts, int* self_idx, Path* paths)
{
    stringstream converter;

    int rover_id = -1;
    size_t idx_start, idx_end;
    idx_start = idx_end = (size_t)(-1);
    double cx, cy, ct, x, y;
    cx = cy = ct = x = y = 0.0;

    converter << msg_parts[0];
    converter >> rover_id;
    converter.str("");
    converter.clear();

    if(rover_id == *self_idx) {
        return;
    }

    converter << msg_parts[1];
    converter >> idx_start;
    converter.str("");
    converter.clear();

    converter << msg_parts[2];
    converter >> idx_end;
    converter.str("");
    converter.clear();

    if(rover_id > 0 && rover_id != *self_idx) {
        paths[rover_id].RemoveRange(idx_start, idx_end);
    }
}
