//
// Created by Mason U'Ren on 2019-02-13.
//

#include "SlamAdapter.h"

void SlamAdapter::updateKinematics(
        const string &rName,
        const Pose &pose,
        const Velocity &vel) {
    ActiveRovers::getInstance()->getRoverByName(rName).updatePoseVel(pose, vel);
}

void SlamAdapter::updateDetections(
        const string &rName,
        const std::array<Sonar, 3> &sonar) {
    ActiveRovers::getInstance()->getRoverByName(rName).updateMLIncidentRay(sonar);
}

void SlamAdapter::recordAuxilaryRoversBelief(
        const string &rName,
        const Pose &pose,
        const float confidence) {
    ActiveRovers::getInstance()->getRoverByName(rName).updateBelief(pose, confidence);
}

void SlamAdapter::slamUpdate(const string &rName) {
    ActiveRovers::getInstance()->getRoverByName(rName).spareExtendedInformationFilter();
}

void SlamAdapter::logAuxilaryFeatureSet(
        const string &rName,
        const std::array<Feature, 3> &features,
        const Classifier &classifier,
        const string &publisher) {
    ActiveRovers::getInstance()->getRoverByName(rName).integrateGlobalFS(features, classifier, publisher);
}

void SlamAdapter::updateTransformationByRover(
        const Pose &transformation,
        const std::string &pairedRover) {
    float xTran = (*this->transformations)[pairedRover]->x_translation->filterValue(transformation.x);
    float yTran = (*this->transformations)[pairedRover]->y_translation->filterValue(transformation.y);
    float oTrain = (*this->transformations)[pairedRover]->orientation->filterValue(transformation.theta);
}

void SlamAdapter::addTransformation(
        const std::string &roverName,
        Transformation *trans) {
    this->transformations->insert(std::make_pair(roverName, trans));
}

void SlamAdapter::addFeatureSet(
        const std::string &roverName,
        Classifier *setClassifier) {
    this->publishedClassifiers->insert(std::make_pair(roverName, setClassifier));
}

Transformation * SlamAdapter::checkTransformation(const std::string &roverName) {
    auto r_copy{roverName};
    try {
        return (*this->transformations).at(lower(r_copy));
    } catch (const std::out_of_range & err) {
        std::ostringstream msg{};
        msg << __PRETTY_FUNCTION__ << std::endl;
        msg << "\t Rover name <" << lower(r_copy) << "> NOT found in SlamAdapter::transformations list." << std::endl;
        msg << "\n" << err.what() << std::endl;
        Logger::getInstance(lower(r_copy))->error(msg.str());
    }

}

Classifier * SlamAdapter::checkFeatureSet(const std::string &roverName) {
    auto r_copy{roverName};
    try {
        return (*this->publishedClassifiers).at(lower(r_copy));
    } catch (const std::out_of_range & err) {
        std::ostringstream msg{};
        msg << __PRETTY_FUNCTION__ << std::endl;
        msg << "\t Rover name <" << lower(r_copy) << "> NOT found in SlamAdapter::publishedClassifiers list." << std::endl;
        msg << "\n" << err.what() << std::endl;
        Logger::getInstance(lower(r_copy))->error(msg.str());
    }

}

