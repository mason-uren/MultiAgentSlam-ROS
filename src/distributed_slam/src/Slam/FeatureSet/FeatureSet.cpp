//
// Created by Mason U'Ren on 2019-04-12.
//

#include "FeatureSet.h"

void FeatureSet::addToSet(const FEATURE &feature, const POSE &rPose) {
    set[currFeatIdx] = feature;
    incidentOrient[currFeatIdx] = rPose.theta;
    incrPtr();
    if (isSetFull()) {
        analyzeFeats();
    }
}

bool FeatureSet::readyToPublish() {
    return isSetFull();
}

std::tuple<std::array<FEATURE, FEATURE_LIMIT>, CLASSIFIER> FeatureSet::publishSet() {
    std::cout << "Publishing Set" << std::endl;
    std::cout << "Area : " << classifier.area << std::endl;
    std::cout << "Orientation : " << classifier.orientation << std::endl;
    std::cout << "Classifier : " << classifier.signature << std::endl;
    return std::tuple<std::array<FEATURE, FEATURE_LIMIT>, CLASSIFIER>{set, classifier};
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
    static bool run = false;
    if (!run) {
        run = true;
    }
    return !static_cast<bool>(currFeatIdx % FEATURE_LIMIT);
}

void FeatureSet::fsArea() {
    std::array<float, FEATURE_LIMIT> legs {
        Equations::getInstance()->distBetweenPts(set[0].pose, set[1].pose),
        Equations::getInstance()->distBetweenPts(set[0].pose, set[2].pose),
        Equations::getInstance()->distBetweenPts(set[1].pose, set[2].pose)
    };
    float area = (legs[0] + legs[1] + legs[2]) / 2;
    classifier.area =  sqrtf(area * (area - legs[0]) * (area - legs[1]) * (area - legs[2]));
}

void FeatureSet::fsOrientation() {
    for (int i = 0; i < FEATURE_LIMIT; i++) {
        classifier.orientation += Equations::getInstance()->wrapTheta(set[i].incidentRay.angle + incidentOrient[i]);
    }

    // To be retreived by `integrateGlobalFS()`
    set[0].pose.theta = classifier.orientation;
}

void FeatureSet::fsSignature() {
    classifier.signature =  Equations::getInstance()->normalizeValue(
            Equations::getInstance()->cantor(classifier.area, classifier.orientation),
            0, SIGNATURE_MAX
    );
}
