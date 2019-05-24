//
// Created by Mason U'Ren on 2019-02-13.
//

#include "SlamAdapter.h"

void SlamAdapter::updateKinematics(const string &rName, const POSE &pose, const VELOCITY &vel) {
    ActiveRovers::getInstance()->getRoverByName(rName).updatePoseVel(pose, vel);
}

void SlamAdapter::updateDetections(const string &rName, const std::array<SONAR, 3> &sonar) {
    ActiveRovers::getInstance()->getRoverByName(rName).updateMLIncidentRay(sonar);
}

void SlamAdapter::recordAuxilaryRoversBelief(const string &rName, const POSE &pose, const float confidence) {
    ActiveRovers::getInstance()->getRoverByName(rName).updateBelief(pose, confidence);
}

void SlamAdapter::slamUpdate(const string &rName) {
    ActiveRovers::getInstance()->getRoverByName(rName).spareExtendedInformationFilter();
}

void SlamAdapter::logAuxilaryFeatureSet(const string &rName, const std::array<FEATURE, 3> &features, const CLASSIFIER &classifier, const string &publisher) {
    ActiveRovers::getInstance()->getRoverByName(rName).integrateGlobalFS(features, classifier, publisher);
}

void SlamAdapter::updateTransformationByRover(const POSE &transformation, const std::string &pairedRover) {
    float xTran = (*this->transformations)[pairedRover]->x_translation->filterValue(transformation.x);
    float yTran = (*this->transformations)[pairedRover]->y_translation->filterValue(transformation.y);
    float oTrain = (*this->transformations)[pairedRover]->orientation->filterValue(transformation.theta);
}

void SlamAdapter::addTransformation(const std::string &roverName, Transformation *trans) {
    this->transformations->insert(std::make_pair(roverName, trans));
}

void SlamAdapter::addFeatureSet(const std::string &roverName, std::tuple<std::array<FEATURE, FEATURE_LIMIT>, CLASSIFIER> *set) {
    this->recentlyPublishedFS->insert(std::make_pair(roverName, set));
}

Transformation* SlamAdapter::checkTransformation(const std::string &roverName) {
    return (*this->transformations)[roverName];
}

std::tuple<std::array<FEATURE, FEATURE_LIMIT>, CLASSIFIER> *SlamAdapter::checkFeatureSet(const std::string &roverName) {
    return (*this->recentlyPublishedFS)[roverName];
}

