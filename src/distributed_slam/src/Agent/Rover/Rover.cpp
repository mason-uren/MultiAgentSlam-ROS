//
// Created by Mason U'Ren on 2019-02-13.
//

#include "Rover.h"

unsigned int Rover::getID() const {
    return static_cast<unsigned int>(*this->ID.data);
}

std::string Rover::getName() const {
    return this->name;
}

VELOCITY * Rover::getVelocity() const {
    return &(*this->vel);
}

POSE * Rover::getCurrentPose() const {
    return &(*this->pose);
}

float Rover::getConfidence() const {
    return this->confidence->getValue();
}

void Rover::addSeif(Seif *seif) {
    if (!this->seif) {
        this->seif = std::shared_ptr<Seif>(seif);
    }
}

void Rover::addDetection(Detection *detection) {
    if (!this->detection) {
        this->detection = std::shared_ptr<Detection>(detection);
    }
}

void Rover::addLocalMap(RedBlackTree *localMap) {
    if (!this->localMap) {
        this->localMap = std::shared_ptr<RedBlackTree>(localMap);
    }
}

void Rover::updatePoseVel(const POSE &pose, const VELOCITY &velocity) {
    this->setVelocity(velocity);
    this->integratePose(pose);
}

void Rover::updateMLIncidentRay(const std::array<SONAR, 3> &sonar) {
    this->detection->MLIncidentRay(sonar);
}

void Rover::updateBelief(const POSE &pose, const float &confidence) {
    this->setCurrentPose(pose);
    this->setConfidence(confidence);
}

void Rover::spareExtendedInformationFilter() {
    this->seif->motionUpdate(*this->vel);
    this->integrateFilteredPose(this->seif->stateEstimateUpdate());
    this->seif->measurementUpdate(*this->detection->getIncidentRay());
    this->seif->sparsification();
}

void Rover::integrateLocalFS(const std::array<FEATURE, FEATURE_LIMIT> &features, const CLASSIFIER &classifier) {
    uint16_t idx;
    if(!this->localMap->findMLClassifier(classifier, idx)) {
        this->localMap->addToTree(classifier, features);
    }
}

void Rover::integrateGlobalFS(const std::array<FEATURE, FEATURE_LIMIT> &foundFeat, const CLASSIFIER &classifier, const string &publisher) {
    uint16_t idx;
    if (this->localMap->findMLClassifier(classifier, idx)) {
        auto features{std::array<FEATURE, FEATURE_LIMIT>()};
        this->localMap->getFeaturesFromNode(features, idx);
        transformation = tuple<POSE, string>(this->estimateMapTransformation(features, foundFeat), publisher);
        this->canPublish = true;
    }
}

bool Rover::readyToPublish() {
    if (this->canPublish) {
        this->canPublish = false;
        return true;
    }
    return false;
}

tuple<POSE, string> Rover::publish() {
    return transformation;
}

void Rover::setName(const std::string &name) {
    this->name = name;
}

void Rover::setVelocity(const VELOCITY &velocity) {
    *this->vel = velocity;
}

void Rover::setCurrentPose(const POSE &belief) {
    lock_guard<mutex> guard(mtx);
    *this->pose = belief;
}

void Rover::setConfidence(const float &confi) {
    lock_guard<mutex> guard(mtx);
    this->confidence->filterValue(confi);
}

void Rover::integratePose(const POSE &pose) {
    this->setCurrentPose(pose);
    this->updateMeans();
    this->updateVariances();
}

void Rover::integrateFilteredPose(const POSE &pose) {
    this->integratePose(pose);
    this->tuneConfi();
}

void Rover::updateMeans() {
    Moments::getInstance()->motion->means[num(pos_val::X)]->onlineAverage(pose->x);
    Moments::getInstance()->motion->means[num(pos_val::Y)]->onlineAverage(pose->y);
    Moments::getInstance()->motion->means[num(pos_val::THETA)]->onlineAverage(pose->theta);
}

void Rover::updateVariances() {
    Moments::getInstance()->motion->variances[num(pos_val::X)]->onlineVariance(
            pose->x,
            Moments::getInstance()->motion->means[num(pos_val::X)]->getFilteredValue());
    Moments::getInstance()->motion->variances[num(pos_val::Y)]->onlineVariance(
            pose->y,
            Moments::getInstance()->motion->means[num(pos_val::Y)]->getFilteredValue());
    Moments::getInstance()->motion->variances[num(pos_val::THETA)]->onlineVariance(
            pose->theta,
            Moments::getInstance()->motion->means[num(pos_val::THETA)]->getFilteredValue());
}

void Rover::tuneConfi() {
    float sum{0};
    for (auto variance : Moments::getInstance()->motion->variances) {
        sum += variance->getFilteredVariance();
    }

    // max sum confi is 3 (ie for <x, y, theta> st. individ confi max is 1)
    this->setConfidence(
            Equations::getInstance()->normalizeValue(sum, 0, MAX_CONFI)
            );
}

POSE Rover::estimateMapTransformation( const array<FEATURE, FEATURE_LIMIT> &fs_1, const array<FEATURE, FEATURE_LIMIT> &fs_2) {
    auto cent_1(Equations::getInstance()->centroid({
        fs_1[0].pose,
        fs_1[1].pose,
        fs_1[2].pose
    }));
    auto cent_2(Equations::getInstance()->centroid({
         fs_2[0].pose,
         fs_2[1].pose,
         fs_2[2].pose
    }));
    auto xyTrans(this->mapTranslation(cent_1, cent_2));
    auto orient(this->mapOrientation(fs_1[0].pose.theta, fs_2[0].pose.theta));
    return POSE{xyTrans.x, xyTrans.y, orient};
}

LOCATION Rover::mapTranslation(const LOCATION &fsCentroid, const LOCATION &otherCentroid) {
    return LOCATION{.x = fsCentroid.x - otherCentroid.x, .y = fsCentroid.y - otherCentroid.y};
}

float Rover::mapOrientation(const float &fsOrientation, const float &otherOrientation) {
    return Equations::getInstance()->wrapTheta(fsOrientation - otherOrientation);
}