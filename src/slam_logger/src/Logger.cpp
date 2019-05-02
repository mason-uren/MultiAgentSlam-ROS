//
// Created by csadmin on 5/22/19.
//
#include "../include/Logger.h"

// TODO
void Logger::report_err(const std::string &message) {
    std::ostringstream msg{"[" + std::to_string(this->getTime()) + "]" + "ERROR: " + message};
    this->writeToFile(this->eLog, msg.str());
}

void Logger::record(const BELIEF &belief) {
    std::ostringstream message{};
    message << "[" << this->getTime() << "]: ";
    message << roverName << ", ";
    message << belief.currentPose.x << ", "
                    << belief.currentPose.y << ", "
                    << belief.currentPose.theta << ", "
                    << belief.roverConfidence << std::endl;
    this->writeToFile(this->beliefLog, message.str());
}

void Logger::record(const std::array<FEATURE, FEATURE_LIMIT> &features, const CLASSIFIER &classifer) {
    std::ostringstream message{};
    message << "[" << this->getTime() << "]: ";
    for (auto feature : features) {
        message << feature.pose.x << ", "
                << feature.pose.y << ", "
                << feature.incidentRay.angle << ", ";
    }
    message << classifer.area << ", "
            << classifer.orientation << ", "
            << classifer.signature << std::endl;
    this->writeToFile(this->featureLog, message.str());
}

void Logger::record(const POSE &transformation, const std::string &targetRover) {
    std::ostringstream message{};
    message << "[" << this->getTime() << "]: ";
    message << this->roverName << ", " << targetRover << ", ";
    message << transformation.x << ", "
            << transformation.y << ", "
            << transformation.theta << std::endl;
    this->writeToFile(this->transformationLog, message.str());
}
void Logger::clearErrorLog() {
    if (this->eLog.is_open()) {
        remove(this->errPath.str().c_str()) ?
            std::cerr << "Error deleting file " << ERR_FILE << "." << std::endl :
            std::cout << "File " << ERR_FILE << " successfully deleted." << std::endl;
    }
    else {
        std::cerr << "File " << ERR_FILE << " NOT found." << std::endl;
    }
}

void Logger::clearInfoLogs() {
    // Beliefs log
    if (this->beliefLog.is_open()) {
        remove(this->belPath.str().c_str()) ?
            std::cerr << "Error deleting file " << BEL_FILE << "." << std::endl :
            std::cout << "File " << BEL_FILE << " successfully deleted." << std::endl;
    }
    else{
        std::cerr << "File " << BEL_FILE << " NOT found." << std::endl;
    }

    // Features log
    if (this->featureLog.is_open()) {
        remove(this->featPath.str().c_str()) ?
        std::cerr << "Error deleting file " << FEAT_FILE << "." << std::endl :
        std::cout << "File " << FEAT_FILE << " successfully deleted." << std::endl;
    }
    else{
        std::cerr << "File " << FEAT_FILE << " NOT found." << std::endl;
    }

    // Transformations log
    if (this->transformationLog.is_open()) {
        remove(this->transPath.str().c_str()) ?
        std::cerr << "Error deleting file " << TRANS_FILE << "." << std::endl :
        std::cout << "File " << TRANS_FILE << " successfully deleted." << std::endl;
    }
    else{
        std::cerr << "File " << TRANS_FILE << " NOT found." << std::endl;
    }
}

long Logger::getTime() {
    return clock() - this->ticks;
}

void Logger::writeToFile(std::ofstream &file, const std::string &message) {
    file << message;
    file.flush();
}
