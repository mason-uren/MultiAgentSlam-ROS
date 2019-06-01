#include "../include/Adapter.h"

using json = nlohmann::json;
using lim_float = std::numeric_limits<float>;

using std::unique_ptr;
using std::make_unique;
using std::string;
using std::get;

namespace fs = boost::filesystem;

void Adapter::loadDefaultConfig() {
    json jsonFileConfig;
    systemConfig = make_unique<SYS_CONFIG_IN>();
    configParser = make_unique<ConfigParser>();
    sharedMemory = make_unique<SharedMemory>();

    this->msg << "Loading Distributed Slam Configuration..." << std::endl;
    if (!configParser->loadJSONFromFile(CONFIG_PATH, &jsonFileConfig)) {
        this->err << CONFIG_PATH << " is missing. \n";
        this->err << "Maybe the path to the JSON config needs to be modified. \n";
        this->err << "Exiting... " << __PRETTY_FUNCTION__ << std::endl;
        Logger::getInstance(*this->roverName)->error(this->err.str());
        exit(EXIT_FAILURE);
    }
    configParser->parseConfig(&(*systemConfig), &jsonFileConfig);
    systemConfig->block_id++;
    sharedMemory->writeMemoryIn(&(*systemConfig));
    this->msg << "Configuration Parsed." << std::endl;
    Logger::getInstance(*this->roverName)->status(msg.str());
    msg.clear();
}

void Adapter::jsonInitialize() {
    // Don't need since we aren't communicating between processes
//    while (true) {
//        if (!sharedMemory->readMemoryIn(&(*systemConfig)) && systemConfig->config.hash != 0) {
//            std::cout << "Configuration Loaded." << std::endl;
//            break;
//        }
//        usleep(500);
//    }

    if (!this->roverName) {
        std::stringstream msg{};
        msg << "Adapter must be aware of rover name before calling `Adapter::jsonInitialize`\n";
        msg << "Check that `Adapter::setRoverName` is being called in the proper execution order.\n";
        msg << "Exiting... " << __PRETTY_FUNCTION__ << std::endl;
        Logger::getInstance(*this->roverName)->error(msg.str());
        exit(EXIT_FAILURE);
    }

    SlamAdapter::getInstance();
    ActiveRovers::getInstance();
    Equations::getInstance();
    Moments::getInstance();
    FeatureSet::getInstance();

    // Need to populate activeRovers and build translation based on number of rovers.
    SLAM_CONFIG slamConfig = systemConfig->config.slamConfig;
    if (slamConfig.valid) {
        for (int rover_id = 0; rover_id < slamConfig.numberOfRovers; rover_id++) {
            ROVER_CONFIG roverConfig = slamConfig.rovers[rover_id];
            if (roverConfig.valid) {
                roverConfig.ID = rover_id;
                auto *rover{new Rover(&roverConfig)};
                DETECTION_CONFIG detectionConfig = slamConfig.detectionConfig;
                SEIF_CONFIG seifConfig = slamConfig.seifConfig;
                LOCAL_MAP_CONFIG localMapConfig = slamConfig.localMapConfig;

                if (*this->roverName == roverConfig.name &&
                    (detectionConfig.valid && seifConfig.valid && localMapConfig.valid)) {
                    detection = unique_ptr<Detection>(new Detection(&detectionConfig));
                    seif = unique_ptr<Seif>(new Seif(&seifConfig));
                    localMap = unique_ptr<RedBlackTree>(new RedBlackTree(&localMapConfig));
                    rover->addDetection(&(*detection));
                    rover->addSeif(&(*seif));
                    rover->addLocalMap(&(*localMap));

                    *this->roverName = rover->getName();
                }

                ActiveRovers::getInstance()->addRover(*rover);
                SlamAdapter::getInstance()->addTransformation(
                        roverConfig.name,
                        new Transformation{});
                SlamAdapter::getInstance()->addFeatureSet(
                        roverConfig.name,
                        new Classifier{}
                );
            }
        }
    }
}

void Adapter::setRoverName(const std::string &name) {
    *this->roverName = name;
}

void Adapter::kinematicHandler(const Pose &pose, const Velocity &vel) {
    SlamAdapter::getInstance()->updateKinematics(*roverName, pose, vel);
}

void Adapter::sonarHandler(const std::array<Sonar, 3> &sonar) {
    SlamAdapter::getInstance()->updateDetections(*this->roverName, sonar);
}

void Adapter::slamHandler() {
    SlamAdapter::getInstance()->slamUpdate(*this->roverName);
}

Belief Adapter::publishBelief() {
    return Belief {
            *ActiveRovers::getInstance()->getRoverByName(*this->roverName).getCurrentPose(),
            ActiveRovers::getInstance()->getRoverByName(*this->roverName).getConfidence()
    };
}

bool Adapter::publishFeatureSet(tuple<array<Feature, 3>, Classifier> *set) {
    bool ready{false};
    if ((ready = FeatureSet::getInstance()->readyToPublish())) {
        *set = FeatureSet::getInstance()->publishSet();
        ActiveRovers::getInstance()->getRoverByName(*roverName).integrateLocalFS(get<0>(*set), get<1>(*set));
    }
    return ready;
}

bool Adapter::publishTransformation(std::tuple<Pose, std::string> *transformation) {
    bool ready{false};
    if ((ready = ActiveRovers::getInstance()->getRoverByName(*roverName).readyToPublish())) {
        *transformation = ActiveRovers::getInstance()->getRoverByName(*this->roverName).publish();

        std::cout << __PRETTY_FUNCTION__ << std::endl;
        std::cout << "\tReceiver (ofPT) : " << *this->roverName << "\tSender : " << get<1>(*transformation) << std::endl;
        // Save transformation locally
        SlamAdapter::getInstance()->updateTransformationByRover(get<0>(*transformation), *this->roverName);
    }
    return ready;
}

void Adapter::auxilaryRoverHandler(const ros_slam_msgs::AuxBeliefs::ConstPtr &auxBeliefs) {
    for (auto i = uint(0); i < auxBeliefs->aux_beliefs.size(); i++) {
        if (!this->isSelf(i)) {
            auto auxRoverName{getRoverName(i)};
            Pose pose{
                auxBeliefs->aux_beliefs[i].pose.x,
                auxBeliefs->aux_beliefs[i].pose.y,
                auxBeliefs->aux_beliefs[i].pose.theta
            };
            if (!this->isSameBelief(auxRoverName, pose)) {
                auto confi{auxBeliefs->aux_beliefs[i].confidence};
                SlamAdapter::getInstance()->recordAuxilaryRoversBelief(auxRoverName, pose, confi);
            }
        }
    }
}

void Adapter::featureSetHandler(const ros_slam_msgs::AuxFeatureSet::ConstPtr &auxFS) {
    for (auto i = uint(0); i < auxFS->aux_feat_sets.size(); i++) {
        if (!this->isSelf(i)) {
            auto auxRoverName{getRoverName(i)};
            auto auxFeatureSet{std::array<Feature, FEATURE_LIMIT>{
                    reinterpret_cast<const Feature &>(auxFS->aux_feat_sets[i].set[0]),
                    reinterpret_cast<const Feature &>(auxFS->aux_feat_sets[i].set[1]),
                    reinterpret_cast<const Feature &>(auxFS->aux_feat_sets[i].set[2])
            }};
            Signature auxSignature{
                    auxFS->aux_feat_sets[i].uuid.signature.mapped_pt,
                    std::vector<float>{
                            auxFS->aux_feat_sets[i].uuid.signature.boundaries[0],
                            auxFS->aux_feat_sets[i].uuid.signature.boundaries[1]
                    }
            };
            auto auxClassifier{
                Classifier{
                        auxFS->aux_feat_sets[i].uuid.area,
                        auxFS->aux_feat_sets[i].uuid.orientation,
                        auxSignature
                }
            };
            if (!this->isSameFeatureSet(auxRoverName, auxClassifier)) {
                SlamAdapter::getInstance()->logAuxilaryFeatureSet(
                        *this->roverName,
                        auxFeatureSet,
                        auxClassifier,
                        auxRoverName
                );
            }
        }
    }
}

/*
 * Visualization
 * 1. Aux-Rover seeing feature set and publishes
 * 2. Rover recognizes feature set
 * 3. Rover estimates transformation between rovers
 * 4. Rover passes estimate back to Aux-Rover
 * 5. Aux-Rover checks partner list looking for itself
 * 6. If see save transformation in location of Rover
 */
void Adapter::transformationHandler(const ros_slam_msgs::TransformationPairs::ConstPtr &transPairs) {
    for (auto i = uint(0); i < transPairs->transformation_sets.size(); i++) {
        if (isSelf(getRoverAddress(transPairs->agent_pairs[i].partner))) {
            auto auxRoverName{getRoverName(i)};
            auto transformation{Pose {
                    transPairs->transformation_sets[i].loc_translation.x,
                    transPairs->transformation_sets[i].loc_translation.y,
                    transPairs->transformation_sets[i].orientation
            }};
            if (!this->isSameTransformation(auxRoverName, transformation)) {
                SlamAdapter::getInstance()->updateTransformationByRover(transformation, auxRoverName);
            }
        }
    }
}

bool Adapter::isSelf(const int &targetRoverIdx) {
    return targetRoverIdx != BAD_ROVER_IDX && targetRoverIdx == getRoverAddress(*this->roverName);
}

bool Adapter::isSameBelief(const std::string &targetRover, const Pose &pose) {
    return *ActiveRovers::getInstance()->getRoverByName(targetRover).getCurrentPose() == pose;
}

// Purposely only checking CLASSIFIER; FEATURES incorporated in-case extra checks are needed
bool Adapter::isSameFeatureSet(const std::string &targetRover,
                               const Classifier &setClassifier) {
    auto featureSetToCheck{SlamAdapter::getInstance()->checkFeatureSet(targetRover)};
    return *featureSetToCheck == setClassifier;
}

bool Adapter::isSameTransformation(const std::string &targetRover, const Pose &trans) {
    auto transToCheck{SlamAdapter::getInstance()->checkTransformation(targetRover)};
    return Pose{
            transToCheck->x_translation->getValue(),
            transToCheck->y_translation->getValue(),
            transToCheck->orientation->getValue()
    } == trans;
}

std::string Adapter::getLocalName() {
    return *this->roverName;
}
