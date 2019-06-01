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

Velocity * Rover::getVelocity() const {
    return &(*this->vel);
}

Pose * Rover::getCurrentPose() const {
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

void Rover::updatePoseVel(const Pose &pose, const Velocity &velocity) {
    this->setVelocity(velocity);
    this->integratePose(pose);
}

void Rover::updateMLIncidentRay(const std::array<Sonar, 3> &sonar) {
    this->detection->MLIncidentRay(sonar);
}

void Rover::updateBelief(const Pose &pose, const float &confidence) {
    this->setCurrentPose(pose);
    this->setConfidence(confidence);
}

void Rover::spareExtendedInformationFilter() {
    this->seif->motionUpdate(*this->vel);
    this->integrateFilteredPose(this->seif->stateEstimateUpdate());
    // TODO testing
    if (this->detection->hasIncidentRay()) {
        this->seif->measurementUpdate(*this->detection->getIncidentRay());
    }
    this->seif->sparsification();
}

void Rover::integrateLocalFS(const std::array<Feature, FEATURE_LIMIT> &features, const Classifier &classifier) {
    uint16_t idx;
    if(!this->localMap->findMLClassifier(classifier, idx)) {
        this->localMap->addToTree(classifier, features);
    }
}

void Rover::integrateGlobalFS(const std::array<Feature, FEATURE_LIMIT> &foundFeat, const Classifier &classifier, const string &publisher) {
    uint16_t idx;
    if (this->localMap->findMLClassifier(classifier, idx)) {
        auto features{std::array<Feature, FEATURE_LIMIT>()};
        this->localMap->getFeaturesFromNode(features, idx);
        transformation = tuple<Pose, string>(this->estimateMapTransformation(features, foundFeat), publisher);
        this->canPublish = true;
    }
}

bool Rover::readyToPublish() {
    if (this->canPublish) {
        return !(this->canPublish = !this->canPublish);
    }
    return this->canPublish;


}

tuple<Pose, string> Rover::publish() {
    return transformation;
}

void Rover::setName(const std::string &name) {
    this->name = name;
}

void Rover::setVelocity(const Velocity &velocity) {
    *this->vel = velocity;
}

void Rover::setCurrentPose(const Pose &belief) {
    lock_guard<mutex> guard(mtx);
    *this->pose = belief;
}

void Rover::setConfidence(const float &confi) {
    lock_guard<mutex> guard(mtx);
    this->confidence->filterValue(confi);
}

void Rover::integratePose(const Pose &pose) {
    this->setCurrentPose(pose);
    this->updateMeans();
    this->updateVariances();
}

void Rover::integrateFilteredPose(const Pose &pose) {
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
    static float lowConfi{std::numeric_limits<float>::max()};
    static float highConfi{std::numeric_limits<float>::min()};
    float sum{0};
    for (auto variance : Moments::getInstance()->motion->variances) {
        sum += variance->getFilteredVariance();
    }

    lowConfi = std::fmin(lowConfi, sum);
    highConfi = std::fmax(sum, highConfi + std::fabs(highConfi - sum));

    std::ostringstream msg{};
    if (debugReport) {
        msg << __PRETTY_FUNCTION__ << std::endl;
        msg << "\t Previous Confidence : " << (*this->confidence).getValue();
    }

    /**
     * Bound values between [0, 1].
     * As variance grows confidence should shrink.
     * As variance shrinks confidence should grow.
     */
    this->setConfidence(
            Equations::getInstance()->normalizeValue(sum, lowConfi, highConfi)
    );

    if (debugReport) {
        msg << "\t New Confidence : " << (*this->confidence).getValue() << std::endl;
        msg << "\t Sum : " << sum << "\t Low-Bound : " << lowConfi << "\t High-Bound : " << highConfi << std::endl;
        msg << "\t X-mean : " << Moments::getInstance()->motion->means[num(pos_val::X)]->getFilteredValue();
        msg << "\t X-var : " << Moments::getInstance()->motion->variances[num(pos_val::X)]->getFilteredVariance() << std::endl;
        msg << "\t Y-mean : " << Moments::getInstance()->motion->means[num(pos_val::Y)]->getFilteredValue();
        msg << "\t Y-var : " << Moments::getInstance()->motion->variances[num(pos_val::Y)]->getFilteredVariance() << std::endl;
        msg << "\t Theta-mean : " << Moments::getInstance()->motion->means[num(pos_val::THETA)]->getFilteredValue();
        msg << "\t Theta-var : " << Moments::getInstance()->motion->variances[num(pos_val::THETA)]->getFilteredVariance() << std::endl;
        Logger::getInstance(this->name)->status(msg.str());
    }
}

Pose Rover::estimateMapTransformation(
        const array<Feature, FEATURE_LIMIT> &fs_1,
        const array<Feature, FEATURE_LIMIT> &fs_2) {
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
    return Pose{xyTrans.x, xyTrans.y, orient};
}

Location Rover::mapTranslation(const Location &fsCentroid, const Location &otherCentroid) {
    return Location{fsCentroid.x - otherCentroid.x, fsCentroid.y - otherCentroid.y};
}

float Rover::mapOrientation(const float &fsOrientation, const float &otherOrientation) {
    return Equations::getInstance()->wrapTheta(fsOrientation - otherOrientation);
}