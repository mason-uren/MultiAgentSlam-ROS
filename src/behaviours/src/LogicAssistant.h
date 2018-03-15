//
// Created by student on 3/14/18.
//
#include "Controller.h"

#ifndef PROJECT_LOGICASSISTANT_H
#define PROJECT_LOGICASSISTANT_H

class LogicAssistant {
private:
    ControllerName prev_state;
    ControllerName curr_state;
    bool cube_held;
    bool cube_seen;
    bool home_seen;
    OBS_TYPE detection;
public:
    LogicAssistant () :
            prev_state(NO_STATE),
            curr_state(NO_STATE),
            cube_held(false),
            cube_seen(false),
            home_seen(false),
            detection(NO_OBSTACLE)
    {}
};

#endif //PROJECT_LOGICASSISTANT_H
