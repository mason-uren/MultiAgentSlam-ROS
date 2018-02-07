//
// Created by student on 2/7/18.
//

#ifndef PROJECT_OBSTACLEASSISTANT_H
#define PROJECT_OBSTACLEASSISTANT_H


/*
 * Types of avoidance
 */
typedef enum {
    NO_OBSTACLE = 0,
    OBS_LEFT,
    OBS_CENTER,
    OBS_RIGHT,
    OBS_LEFT_CENTER,
    OBS_RIGHT_CENTER,
    HOME
} OBS_TYPE;

/*
 * Sonar Distiguisher
 */
typedef enum {
    LEFT = 0,
    CENTER,
    RIGHT
} SONAR;

typedef struct {
    OBS_TYPE type;
    bool good_detection;
    double detection_dist;
    double crct_ang;
    union {
        std::vector<float> *sonar_left;
        std::vector<float> *sonar_center;
        std::vector<float> *sonar_right;
    };
} DETECTIONS;

class ObstacleAssistant {
private:
    std::map<SONAR, DETECTIONS> *obstacle;
    DETECTIONS detections;


public:

    ObstacleAssistant() : {
            // Create map
            this->obstacle = new std::map<SONAR, DETECTIONS>(),
            // Initialize sonar structures
            this->detections.sonar_left = new std::vector<float>(),
            this->detections.sonar_center = new std::vector<float>(),
            this->detections.sonar_right = new std::vector<float>(),

            this->detections.type = NO_OBSTACLE,
            this->detections.good_detection = false,
            this->detections.detection_dist = 0,
            this->detections.crct_ang = 0,

            this->obstacle[LEFT] = ;

    };


};


#endif //PROJECT_OBSTACLEASSISTANT_H
