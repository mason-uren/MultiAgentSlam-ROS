//
// Created by Mason U'Ren on 2019-02-13.
//

#ifndef C_SLAMADAPTER_H
#define C_SLAMADAPTER_H

#include <include/SharedMemoryStructs.h>
#include <include/SLAMConfigIn.h>
#include <include/Transformation/Transformation.h>

#include "../../Agent/Rover/Rover.h"
#include "../../Utilities/ActiveRovers/ActiveRovers.h"


using std::string;

class SlamAdapter {
public:
    static SlamAdapter *getInstance() {
        static SlamAdapter instance;
        return &instance;
    }
    ~SlamAdapter() = default;
    SlamAdapter(SlamAdapter const&) = delete;
    void operator=(SlamAdapter const&) = delete;

    void updateKinematics(const string &rName, const Pose &pose, const Velocity &vel);
    void updateDetections(const string &rName, const std::array<Sonar, 3> &sonar);
    void recordAuxilaryRoversBelief(const string &rName, const Pose &pose, float confidence);

    void slamUpdate(const string &rName);
    void logAuxilaryFeatureSet(
            const string &rName,
            const std::array<Feature, 3> &features,
            const Classifier &classifier,
            const string &publisher);

    void updateTransformationByRover(const Pose &transformation, const std::string &pairedRover);
    void addTransformation(const std::string &roverName, Transformation *trans);
    void addFeatureSet(const std::string &roverName, Classifier *setClassifier);

    // TODO both need to be re-written
    Transformation* checkTransformation(const std::string &roverName);
    Classifier * checkFeatureSet(const std::string &roverName);

    // FOR TESTING PURPOSES
    std::unordered_map<std::string, Transformation *> *getTransformations() {
        return transformations.get();
    }

private:
    SlamAdapter() :
//            rover(std::shared_ptr<Rover>(new Rover())),
        transformations(new std::unordered_map<std::string, Transformation *>{}),
        publishedClassifiers(new std::unordered_map<std::string, Classifier *>{})
    {}

    /**
     * Variables
     */
//    std::shared_ptr<Rover> rover;
    std::shared_ptr<std::unordered_map<std::string, Transformation *>> transformations;
    std::shared_ptr<std::unordered_map<std::string, Classifier *>> publishedClassifiers;
};


#endif //C_SLAMADAPTER_H
