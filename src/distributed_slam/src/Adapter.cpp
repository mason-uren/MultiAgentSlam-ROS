#include "Adapter.h"

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

    std::cout << "Loading Distributed Slam Configuration..." << std::endl;
    if (!configParser->loadJSONFromFile(CONFIG_PATH, &jsonFileConfig)) {
        std::stringstream root;
        root << CONFIG_PATH << " is missing. \n";
        root << "Maybe the path to the JSON config needs to be modified. \n";
        std::cerr << root.str() << strerror(errno);
        exit(EXIT_FAILURE);
    }
    configParser->parseConfig(&(*systemConfig), &jsonFileConfig);
    systemConfig->block_id++;
    sharedMemory->writeMemoryIn(&(*systemConfig));
    std::cout << "Configuration Parsed." << std::endl;
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
        std::cerr << "Adapter must be aware of rover name before calling `Adapter::jsonInitialize`" << std::endl;
        std::cerr << "Check that `Adapter::setRoverName` is being called in the proper execution order." << std::endl;
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
                    detection = make_unique<Detection>(&detectionConfig);
                    seif = make_unique<Seif>(&seifConfig);
                    localMap = make_unique<RedBlackTree>(&localMapConfig);
                    rover->addDetection(&(*detection));
                    rover->addSeif(&(*seif));
                    rover->addLocalMap(&(*localMap));

                    *this->roverName = rover->getName();
                }

                ActiveRovers::getInstance()->addRover(*rover);
                SlamAdapter::getInstance()->addTransformation(roverConfig.name, new Transformation());
                SlamAdapter::getInstance()->addFeatureSet(roverConfig.name, new std::tuple<std::array<FEATURE, FEATURE_LIMIT>, CLASSIFIER>);
            }
        }
    }
}

void Adapter::setRoverName(const std::string &name) {
    *this->roverName = name;
}

void Adapter::kinematicHandler(const POSE &pose, const VELOCITY &vel) {
    SlamAdapter::getInstance()->updateKinematics(*roverName, pose, vel);
}

void Adapter::sonarHandler(const std::array<SONAR, 3> &sonar) {
    SlamAdapter::getInstance()->updateDetections(*this->roverName, sonar);
}

void Adapter::slamHandler() {
    SlamAdapter::getInstance()->slamUpdate(*this->roverName);
}

BELIEF Adapter::publishBelief() {
    return BELIEF {
            .currentPose = *ActiveRovers::getInstance()->getRoverByName(*this->roverName).getCurrentPose(),
            .roverConfidence = ActiveRovers::getInstance()->getRoverByName(*this->roverName).getConfidence()
    };
}

bool Adapter::publishFeatureSet(tuple<array<FEATURE, 3>, CLASSIFIER> *set) {
    bool ready{false};
    if ((ready = FeatureSet::getInstance()->readyToPublish())) {
        *set = FeatureSet::getInstance()->publishSet();
        ActiveRovers::getInstance()->getRoverByName(*roverName).integrateLocalFS(get<0>(*set), get<1>(*set));
    }
    return ready;
}

bool Adapter::publishTransformation(std::tuple<POSE, std::string> *transformation) {
    bool ready{false};
    if ((ready = ActiveRovers::getInstance()->getRoverByName(*roverName).readyToPublish())) {
        *transformation = ActiveRovers::getInstance()->getRoverByName(*this->roverName).publish();
        // Save transformation locally
        SlamAdapter::getInstance()->updateTransformationByRover(get<0>(*transformation), *this->roverName);
    }
    return ready;
}

void Adapter::auxilaryRoverHandler(const ros_slam_msgs::AuxBeliefs::ConstPtr &auxBeliefs) {
    for (auto i = uint(0); i < auxBeliefs->aux_beliefs.size(); i++) {
        if (!this->isSelf(i)) {
            auto auxRoverName{getRoverName(i)};
            POSE pose{
                .x = auxBeliefs->aux_beliefs[i].pose.x,
                .y = auxBeliefs->aux_beliefs[i].pose.y,
                .theta = auxBeliefs->aux_beliefs[i].pose.theta
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
            auto auxFeatureSet{std::array<FEATURE, FEATURE_LIMIT>{
                    reinterpret_cast<const FEATURE &>(auxFS->aux_feat_sets[i].set[0]),
                    reinterpret_cast<const FEATURE &>(auxFS->aux_feat_sets[i].set[1]),
                    reinterpret_cast<const FEATURE &>(auxFS->aux_feat_sets[i].set[2])
            }};
            auto auxClassifier{CLASSIFIER{
                    .area = auxFS->aux_feat_sets[i].uuid.area,
                    .orientation = auxFS->aux_feat_sets[i].uuid.orientation,
                    .signature = auxFS->aux_feat_sets[i].uuid.signature
            }};
            std::tuple<std::array<FEATURE, FEATURE_LIMIT>, CLASSIFIER> targetFS{auxFeatureSet, auxClassifier};
            if (!this->isSameFeatureSet(auxRoverName, targetFS)) {
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
            auto transformation{POSE {
                    .x = transPairs->transformation_sets[i].loc_translation.x,
                    .y = transPairs->transformation_sets[i].loc_translation.y,
                    .theta = transPairs->transformation_sets[i].orientation
            }};
            if (!this->isSameTransformation(auxRoverName, transformation)) {
                SlamAdapter::getInstance()->updateTransformationByRover(transformation, auxRoverName);
            }
        }
    }
}

bool Adapter::isSelf(const uint16_t &targetRoverIdx) {
    return targetRoverIdx == getRoverAddress(*this->roverName);
}

bool Adapter::isSameBelief(const std::string &targetRover, const POSE &pose) {
    return *ActiveRovers::getInstance()->getRoverByName(targetRover).getCurrentPose() == pose;
}

// Purposely only checking CLASSIFIER; FEATURES incorporated incase extra checks are needed
bool Adapter::isSameFeatureSet(const std::string &targetRover,
                               const std::tuple<std::array<FEATURE, FEATURE_LIMIT>, CLASSIFIER> &set) {
    auto featureSetToCheck{SlamAdapter::getInstance()->checkFeatureSet(targetRover)};
    return get<1>(*featureSetToCheck) == get<1>(set);
}

bool Adapter::isSameTransformation(const std::string &targetRover, const POSE &trans) {
    auto transToCheck{SlamAdapter::getInstance()->checkTransformation(targetRover)};
    return POSE{
            .x = transToCheck->x_translation->getValue(),
            .y = transToCheck->y_translation->getValue(),
            .theta = transToCheck->orientation->getValue()
    } == trans;
}

std::string Adapter::getLocalName() {
    return *this->roverName;
}
