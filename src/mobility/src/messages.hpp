#ifndef MESSAGES_HPP
#define MESSAGES_HPP

#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose2D.h>

#define STATE_MACHINE_TRANSFORM	0
#define STATE_MACHINE_ROTATE	1
#define STATE_MACHINE_TRANSLATE	2

//STL data types
#include <vector>
#include <sstream>

#include "path.hpp"

using namespace csuci;

void messageHandler(const std_msgs::String::ConstPtr& message, int* self_idx, std::string* memberNames, const std::string& publishedName, int& swarmSize,
                    std_msgs::Int16& targetCollected, bool* targetsDetected, geometry_msgs::Pose2D* targetPositions, Path* paths, int& stateMachineState,
                    geometry_msgs::Pose2D& goalLocation, geometry_msgs::Pose2D& currentLocation);
void identityMessage(std::vector<std::string> msg_parts, int* self_idx, std::string* memberNames, const std::string& publishedName, int& swarmSize);
void detectedMessage(std::vector<std::string> msg_parts, std_msgs::Int16& targetCollected, bool* targetsDetected, geometry_msgs::Pose2D* targetPositions,
                     Path& path, int& stateMachineState, geometry_msgs::Pose2D& goalLocation, geometry_msgs::Pose2D& currentLocation);
void pointAddedMessage(std::vector<std::string> msgParts, geometry_msgs::Pose2D& currentLocation, int* self_idx, Path* paths);
void pointRemovedMessage(std::vector<std::string> msgParts, int* self_idx, Path* paths);

#endif
