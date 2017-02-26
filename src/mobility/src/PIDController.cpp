#include "PIDController.h"
#include <math.h>
#include <angles/angles.h>

PIDController::PIDController()
{
    resetTranslationalIntegrator();
    translational_error_prior = 0.0;
}

float PIDController::calculateTranslationalVelocity(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D goalLocation)
{
    float velocity_error = hypot(goalLocation.x-currentLocation.x, goalLocation.y-currentLocation.y);
    translational_integrator = translational_integrator + velocity_error;
    float velocity_derivative = (velocity_error - translational_error_prior);
    float velocity_output = (Kp_T*velocity_error) + (Ki_T*translational_integrator) + (Kd_T*velocity_derivative);
    translational_error_prior = velocity_error;

    if(velocity_output > MAX_LINEAR_VELOCITY)
    {
        velocity_output = MAX_LINEAR_VELOCITY;
    }
    else if (velocity_output < -MAX_LINEAR_VELOCITY)
    {
        velocity_output = -MAX_LINEAR_VELOCITY;
    }

    return velocity_output;


}

void PIDController::resetTranslationalIntegrator()
{
    translational_integrator = 0.0;
}
