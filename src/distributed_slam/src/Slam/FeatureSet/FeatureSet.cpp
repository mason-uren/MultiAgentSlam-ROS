//
// Created by Mason U'Ren on 2019-04-12.
//

#include "FeatureSet.h"

void FeatureSet::addToSet(const Feature &feature, const Pose &rPose) {
    std::cout << "Adding Feature " << currFeatIdx << "!!!" << std::endl;
    set[currFeatIdx] = feature;
    incidentOrient[currFeatIdx] = rPose.theta;
    incrPtr();
    if (isSetFull()) {
        std::cout << "Feature set FULL" << std::endl;
        analyzeFeats();
    }
}

bool FeatureSet::readyToPublish() {
    return isSetFull();
}

std::tuple<std::array<Feature, FEATURE_LIMIT>, Classifier> FeatureSet::publishSet() {
    return std::tuple<std::array<Feature, FEATURE_LIMIT>, Classifier>{set, classifier};
}

void FeatureSet::incrPtr() {
    ++currFeatIdx %= FEATURE_LIMIT;
}

void FeatureSet::analyzeFeats() {
    fsArea();
    fsOrientation();
    fsSignature();
}

bool FeatureSet::isSetFull() {
    static bool procStarted{false};
    if (!procStarted) {
        procStarted = static_cast<bool>(currFeatIdx % FEATURE_LIMIT);
    }
    return !static_cast<bool>(currFeatIdx % FEATURE_LIMIT) && procStarted;
}

void FeatureSet::fsArea() {
    std::array<float, FEATURE_LIMIT> legs {
        Equations::getInstance()->distBetweenPts(set[0].pose, set[1].pose),
        Equations::getInstance()->distBetweenPts(set[0].pose, set[2].pose),
        Equations::getInstance()->distBetweenPts(set[1].pose, set[2].pose)
    };

    /**
     * Area of triangle from sides.
     * Heron's Formula
     */
    float splitPerimeter = (legs[0] + legs[1] + legs[2]) / 2;
    classifier.area =  sqrtf(splitPerimeter * (splitPerimeter - legs[0]) * (splitPerimeter - legs[1]) * (splitPerimeter - legs[2]));
}

void FeatureSet::fsOrientation() {
    for (int i = 0; i < FEATURE_LIMIT; i++) {
        classifier.orientation += Equations::getInstance()->wrapTheta(set[i].incidentRay.angle + incidentOrient[i]);
    }

    // To be retreived by `integrateGlobalFS()`
    set[0].pose.theta = classifier.orientation;
}

void FeatureSet::fsSignature() {
//    static float signLowBound{std::numeric_limits<float>::max()};
//    static float signHighBound{std::numeric_limits<float>::min()};

    auto signature{Equations::getInstance()->szudzikMapping(classifier.area, classifier.orientation)};
//    signLowBound = std::fmin(signLowBound, signature);
//    signHighBound = std::fmax(signature, signHighBound);
//
    classifier.signature = signature;

//            Equations::getInstance()->normalizeValue(
//            signature, std::numeric_limits<float>::min(), std::numeric_limits<float>::max()
//    );
}
