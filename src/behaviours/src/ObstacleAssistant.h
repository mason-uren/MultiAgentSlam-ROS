//
// Created by student on 2/7/18.
//

#ifndef PROJECT_OBSTACLEASSISTANT_H
#define PROJECT_OBSTACLEASSISTANT_H


typedef enum {
    NO_OBSTACLE = 0,
    OBS_LEFT,
    OBS_CENTER,
    OBS_RIGHT,
    OBS_LEFT_CENTER,
    OBS_RIGHT_CENTER,
    HOME
} OBS_TYPE;

typedef enum {
    LEFT = 0,
    CENTER,
    RIGHT
} SONAR;

class ObstacleAssistant {
private:
    typedef struct {
        bool init_detection;
        bool good_detection;
        double smallest_detection;
        double crct_agl;
    } DETECTIONS;

public:
    SONAR type;
    DETECTIONS detections;
    std::vector<float> *monitor;

    explicit ObstacleAssistant(SONAR sonar) : type(sonar),
                          monitor(new std::vector<float>),
                          detections({false, false, 10, 0}) {};
};


#endif //PROJECT_OBSTACLEASSISTANT_H
