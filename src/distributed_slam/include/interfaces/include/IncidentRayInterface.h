//
// Created by Mason U'Ren on 2019-02-13.
//

#ifndef C_INCIDENTRAY_H
#define C_INCIDENTRAY_H

#include <include/SharedMemoryStructs.h>

class IncidentRayInterface {
public:
    virtual ~IncidentRayInterface() = default;

    virtual bool hasIncidentRay() = 0;
    virtual Ray *getIncidentRay() = 0;
    virtual void setIncidentRay(Ray ray) = 0;
};

#endif //C_INCIDENTRAY_H
