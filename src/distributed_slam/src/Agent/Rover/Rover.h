#include <utility>

#include <utility>

//
// Created by Mason U'Ren on 2019-02-13.
//

#ifndef C_ROVER_H
#define C_ROVER_H

#define BOOST_ALLOW_DEPRECATED_HEADERS

#include <functional>
#include <include/SLAMConfigIn.h>
#include <include/RoverInterface.h>
#include <boost/uuid/uuid_generators.hpp>

#include "../../Slam/Seif/Seif.h"
#include "../../Utilities/RedBlackTree/RedBlackTree.h"
#include "../../Agent/Detections/Detection.h"
#include "../../Agent/Moments/Moments.h"

using std::array;
using std::string;
using std::tuple;
using std::get;
using std::accumulate;
using std::mutex;
using std::lock;
using std::lock_guard;
using std::shared_ptr;
using std::make_shared;

constexpr int MAX_CONFI = 3;

class Rover : virtual public RoverInterface {
public:
    explicit Rover(std::string name = "DUMMY") :
        ID(boost::uuids::random_generator()()),
        name(std::move(name)),
        buffer(array<float, FILTER_LENGTH>()),
        confidence(new FIRFilter<float, FILTER_LENGTH>(reinterpret_cast<float (&)[64]>(buffer))),
        pose(new POSE {.x = 0, .y = 0, .theta = 0})
    {}

    explicit Rover(ROVER_CONFIG *roverConfig) :
        ID(boost::uuids::random_generator()()),
        name(roverConfig->name),
        buffer(array<float, FILTER_LENGTH>()),
        confidence(new FIRFilter<float, FILTER_LENGTH>(reinterpret_cast<float (&)[64]>(buffer))),
        pose(new POSE {.x = 0, .y = 0, .theta = 0}),
        vel(new VELOCITY {.linear = 0, .angular = 0})
    {}
    ~Rover() override = default;

//    Rover& operator=(const Rover &rover) {
//        if (this != &rover) {
//            lock(mtx, rover.mtx);
//            lock_guard<mutex> lhs_lk(mtx, std::adopt_lock);
//            lock_guard<mutex> rhs_lk(rover.mtx, std::adopt_lock);
//
//            // Not sure if this is the best imp
//            ID = rover.ID;
//            name = rover.name;
//            buffer = rover.buffer;
//            confidence = rover.confidence;
//            pose = rover.pose;
//            vel = rover.vel;
//            seif = rover.seif;
//            detection = rover.detection;
//            localMap = rover.localMap;
//            transformation = rover.transformation;
//            canPublish = rover.canPublish;
//        }
//        return *this;
//    }

    unsigned int getID() const override;
    std::string getName() const override;
    POSE *getCurrentPose() const override;
    VELOCITY *getVelocity() const override;
    float getConfidence() const override;

    // Allocate Memory, should only be used during initialization.
    void addSeif(Seif *seif);
    void addDetection(Detection *detection);
    void addLocalMap(RedBlackTree *localMap);

    void updatePoseVel(const POSE &pose, const VELOCITY &velocity);
    void updateMLIncidentRay(const std::array<SONAR, RANGE_SENSOR_COUNT> &sonar);
    void updateBelief(const POSE &pose, const float &confidence);
    void spareExtendedInformationFilter();
    void integrateLocalFS(const std::array<FEATURE, FEATURE_LIMIT> &features,
            const CLASSIFIER &classifier);
    void integrateGlobalFS(const std::array<FEATURE, FEATURE_LIMIT> &features,
            const CLASSIFIER &classifier,
            const string &publisher);
    bool readyToPublish();
    tuple<POSE, string> publish();

    // For testing purposes only!
    RedBlackTree *getLocalMap() {
        return &(*localMap);
    }
    Detection *getDetections() {
        return &(*detection);
    }

private:
    void setName(const string &name) override;
    void setCurrentPose(const POSE &belief) override;
    void setVelocity(const VELOCITY &velocity) override;
    void setConfidence(const float &confi) override;

    void integratePose(const POSE &pose);
    void integrateFilteredPose(const POSE &pose);
    void updateMeans();
    void updateVariances();
    void tuneConfi();
    POSE estimateMapTransformation(
            const array<FEATURE, FEATURE_LIMIT> &fs_1,
            const array<FEATURE, FEATURE_LIMIT> &fs_2);
    LOCATION mapTranslation(const LOCATION &fsCentroid, const LOCATION &otherCentroid);
    float mapOrientation(const float &fsOrientation, const float &otherOrientation);

    /**
     * Variables
     */
    mutable mutex mtx;
    boost::uuids::uuid ID;
    string name;
    array<float, FILTER_LENGTH> buffer;
    shared_ptr<FIRFilter<float, FILTER_LENGTH>> confidence;
    shared_ptr<POSE> pose;
    shared_ptr<VELOCITY> vel;
    shared_ptr<Seif> seif;
    shared_ptr<Detection> detection;
    shared_ptr<RedBlackTree> localMap;

    tuple<POSE, string> transformation{};
    bool canPublish{};

    static bool writingPose;
};


#endif //C_ROVER_H
