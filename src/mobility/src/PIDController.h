#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <geometry_msgs/Pose2D.h>

/**
 * This class implements a PID controller for the rovers. The code
 * here should not be modified.
 */
class PIDController
{

public:

    PIDController();
    float calculateTranslationalVelocity(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D goalLocation);
    void resetTranslationalIntegrator();

private:
    float translational_integrator;
    float translational_error_prior;

    static const float Kp_T = 1.5;
    static const float Ki_T = 0.001;
    static const float Kd_T = 0.0;

    static const float MAX_LINEAR_VELOCITY = 0.3;
};

#endif // PIDCONTROLLER_H
