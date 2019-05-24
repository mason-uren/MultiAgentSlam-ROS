//
// Created by Mason U'Ren on 2019-02-13.
//

#ifndef C_ROVERINTERFACE_H
#define C_ROVERINTERFACE_H

#include <string>
#include <include/SharedMemoryStructs.h>

class RoverInterface {
public:
    virtual ~RoverInterface() = default;

    virtual unsigned int getID() const = 0;
    virtual std::string getName() const = 0;
    virtual POSE *getCurrentPose() const = 0;
    virtual VELOCITY *getVelocity() const = 0;
    virtual float getConfidence() const = 0;

    virtual void setName(const std::string &name) = 0;
    virtual void setCurrentPose(const POSE &pose) = 0;
    virtual void setVelocity(const VELOCITY &velocity) = 0;
    virtual void setConfidence(const float &confi) = 0;
};

#endif //C_ROVERINTERFACE_H
