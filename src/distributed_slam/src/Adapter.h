//
// Created by Mason U'Ren on 2019-04-15.
//

#ifndef MULTIAGENTSLAM_ADAPTER_H
#define MULTIAGENTSLAM_ADAPTER_H

#include <memory>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <cmath>
#include <Eigen/Dense>
#include <boost/filesystem.hpp>
#include <boost/operators.hpp>
#include <boost/filesystem/fstream.hpp>

#include <include/SharedMemoryStructs.h>
#include <include/SharedMemory/SharedMemory.h>
#include <include/Tools/cpp14_utils.h>

#include <ros_slam_msgs/AuxBeliefs.h>
#include <ros_slam_msgs/AuxFeatureSet.h>
#include <ros_slam_msgs/TransformationPairs.h>

#include "Agent/Detections/Detection.h"
#include "Slam/SlamAdapter/SlamAdapter.h"
#include "Slam/Seif/Seif.h"
#include "Utilities/ConfigParser/ConfigParser.h"
#include "Utilities/RedBlackTree/RedBlackTree.h"

class Adapter {
public:
    static Adapter *getInstance() {
        static Adapter instance;
        return &instance;
    }
    ~Adapter() = default;
    Adapter(const Adapter &) = delete;
    void operator=(const Adapter &) = delete;

    void setRoverName(const std::string &name);
    void loadDefaultConfig();
    void jsonInitialize();
    void kinematicHandler(const POSE &pose, const VELOCITY &vel);
    void sonarHandler(const std::array<SONAR, 3> &sonar);

    void slamHandler();

    void auxilaryRoverHandler(const ros_slam_msgs::AuxBeliefs::ConstPtr &auxBeliefs);
    void featureSetHandler(const ros_slam_msgs::AuxFeatureSet::ConstPtr &auxFS);
    void transformationHandler(const ros_slam_msgs::TransformationPairs::ConstPtr &transPairs);

    BELIEF publishBelief();
    bool publishFeatureSet(tuple<array<FEATURE, 3>, CLASSIFIER> *set);
    bool publishTransformation(std::tuple<POSE, std::string> *transformation);

    std::string getLocalName();



private:
    Adapter() :
        roverName(new std::string())
    {};

    bool isSelf(const uint16_t &targetRoverIdx);
    bool isSameBelief(const std::string &targetRover, const POSE &pose);
    bool isSameFeatureSet(const std::string &targetRover,
                          const std::tuple<std::array<FEATURE, FEATURE_LIMIT>, CLASSIFIER> &set);
    bool isSameTransformation(const std::string &targetRover, const POSE &trans);

    std::unique_ptr<SharedMemory> sharedMemory;
    std::unique_ptr<ConfigParser> configParser;
    std::unique_ptr<Seif> seif;
    std::unique_ptr<Detection> detection;
    std::unique_ptr<RedBlackTree> localMap;
    std::unique_ptr<SYS_CONFIG_IN> systemConfig;

    std::unique_ptr<std::string> roverName;
//    bool canReadLocation{true};
};

#endif //MULTIAGENTSLAM_ADAPTER_H
