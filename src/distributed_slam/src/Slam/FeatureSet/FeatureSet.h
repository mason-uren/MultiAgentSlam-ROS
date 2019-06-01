//
// Created by Mason U'Ren on 2019-04-12.
//

#ifndef MULTIAGENTSLAM_FEATURESET_H
#define MULTIAGENTSLAM_FEATURESET_H

#include <iostream>
#include <array>
#include <tuple>

#include <include/SharedMemoryStructs.h>
#include <Logger.h>

#include "../../Utilities/Equations/Equations.h"

inline int idx(const int &i) { return i % FEATURE_LIMIT; };

class FeatureSet {
public:
    static FeatureSet *getInstance() {
        static FeatureSet instance;
        return &instance;
    }
    ~FeatureSet() = default;
    FeatureSet(const FeatureSet &) = delete;
    void operator=(const FeatureSet &) = delete;

//    bool operator==(const FeatureSet &rhs) const {
//        return classifier == rhs.classifier;
//    }

    void addToSet(const Feature &feature, const Pose &rPose);
    bool readyToPublish();
    std::tuple<std::array<Feature, FEATURE_LIMIT>, Classifier> publishSet();

private:
    FeatureSet() :
            set(std::array<Feature, FEATURE_LIMIT>{Feature{}, Feature{}, Feature{}}),
            incidentOrient(std::array<float, FEATURE_LIMIT>{}),
            classifier(Classifier{}),
            currFeatIdx(0)
    {}

    void incrPtr();
    void analyzeFeats();
    bool isSetFull();
    void fsArea();
    void fsOrientation();
    void fsSignature();

    std::array<Feature, FEATURE_LIMIT> set;
    std::array<float, FEATURE_LIMIT> incidentOrient;
    Classifier classifier; // TODO classifier may have to be reset each time a feature set is built
    int currFeatIdx;


};


#endif //MULTIAGENTSLAM_FEATURESET_H
