#ifndef SEARCH_CONTROLLER
#define SEARCH_CONTROLLER

#include <geometry_msgs/Pose2D.h>
#include <random_numbers/random_numbers.h>
#include <string>
#include <stack>          // std::stack

/**
 * This class implements the search control algorithm for the rovers. The code
 * here should be modified and enhanced to improve search performance.
 */
class SearchController {

  public:

    SearchController();

    // performs search pattern
    geometry_msgs::Pose2D search(geometry_msgs::Pose2D currentLocation);

    // continues search pattern after interruption
    geometry_msgs::Pose2D continueInterruptedSearch(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D oldGoalLocation);

    // search using waypoints
    void waypointSearchFound(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D oldGoalLocation,std::string botName);

    //
    geometry_msgs::Pose2D waypointNextLocation(geometry_msgs::Pose2D currentLocation ,std::string botName);

    void waypointObstacleAvoidance(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D oldGoalLocation, geometry_msgs::Pose2D alternativeLocation, std::string botName);

  private:

    random_numbers::RandomNumberGenerator* rng;
    geometry_msgs::Pose2D GetNewGoalLocation(std::stack<geometry_msgs::Pose2D> &wp_stack, geometry_msgs::Pose2D currentLocation);

    //Initial waypoints
    static const float d = 0.5;

    static const double final_boundary = 11.0;
    //    float waypoints_x_final [] {final_boundary-d, -(final_boundary-d), -(final_boundary-d),    final_boundary-d,   final_boundary-d, -(final_boundary-2*d), -(final_boundary-2*d),    final_boundary-2*d, final_boundary-2*d, -(final_boundary-3*d), -(final_boundary-3*d),    final_boundary-3*d, final_boundary-3*d, -(final_boundary-4*d), -(final_boundary-4*d),   final_boundary-4*d, final_boundary-4*d};
    //    float waypoints_y_final [] {final_boundary-d,    final_boundary-d, -(final_boundary-d), -(final_boundary-d), final_boundary-2*d,    final_boundary-2*d, -(final_boundary-2*d), -(final_boundary-2*d), final_boundary-3*d,    final_boundary-3*d, -(final_boundary-3*d), -(final_boundary-3*d), final_boundary-4*d,    final_boundary-4*d, -(final_boundary-4*d), -(final_boundary-4*d), final_boundary-5*d};

    //    float waypoints_x2_final [] {final_boundary-4*d, -(final_boundary-5*d), -(final_boundary-5*d),    final_boundary-5*d,    final_boundary-5*d, -(final_boundary-6*d), -(final_boundary-6*d),    final_boundary-6*d, final_boundary-6*d,  -(final_boundary-7*d), -(final_boundary-7*d),    final_boundary-7*d, final_boundary-7*d, -(final_boundary-8*d), -(final_boundary-8*d),    final_boundary-8*d, final_boundary-8*d, final_boundary-8*d, -(final_boundary-9*d), -(final_boundary-9*d),   final_boundary-9*d};
    //    float waypoints_y2_final [] {final_boundary-5*d,    final_boundary-5*d, -(final_boundary-5*d), -(final_boundary-5*d),    final_boundary-6*d,    final_boundary-6*d, -(final_boundary-6*d), -(final_boundary-6*d), final_boundary-7*d,     final_boundary-7*d, -(final_boundary-7*d), -(final_boundary-7*d), final_boundary-8*d,    final_boundary-8*d, -(final_boundary-8*d), -(final_boundary-8*d), final_boundary-9*d, final_boundary-9*d,    final_boundary-9*d, -(final_boundary-9*d), -(final_boundary-9*d)};

    //    float waypoints_x3_final [] { final_boundary-9*d,  -(final_boundary-10*d), -(final_boundary-10*d),   final_boundary-10*d,  final_boundary-10*d, -(final_boundary-11*d), -(final_boundary-11*d),    final_boundary-11*d, final_boundary-11*d, -(final_boundary-12*d), -(final_boundary-12*d),    final_boundary-12*d, final_boundary-12*d, -(final_boundary-13*d), -(final_boundary-13*d),    final_boundary-13*d, final_boundary-13*d, -(final_boundary-14*d), -(final_boundary-14*d),    final_boundary-14*d, final_boundary-14*d, -(final_boundary-15*d), -(final_boundary-15*d),    final_boundary-15*d, final_boundary-15*d, -(final_boundary-16*d), -(final_boundary-16*d),    final_boundary-16*d, final_boundary-16*d,  -(final_boundary-17*d), -(final_boundary-17*d),    final_boundary-17*d, final_boundary-17*d, -(final_boundary-18*d), -(final_boundary-18*d),    final_boundary-18*d, final_boundary-18*d,  -(final_boundary-19*d), -(final_boundary-19*d),    final_boundary-19*d, final_boundary-19*d};
    //    float waypoints_y3_final [] {final_boundary-10*d,     final_boundary-10*d, -(final_boundary-10*d), -(final_boundary-10*d), final_boundary-11*d,    final_boundary-11*d, -(final_boundary-11*d), -(final_boundary-11*d), final_boundary-12*d,    final_boundary-12*d, -(final_boundary-12*d), -(final_boundary-12*d), final_boundary-13*d,    final_boundary-13*d, -(final_boundary-13*d), -(final_boundary-13*d), final_boundary-14*d,    final_boundary-14*d, -(final_boundary-14*d), -(final_boundary-14*d), final_boundary-15*d,    final_boundary-15*d, -(final_boundary-15*d), -(final_boundary-15*d), final_boundary-16*d,    final_boundary-16*d, -(final_boundary-16*d), -(final_boundary-16*d), final_boundary-17*d,     final_boundary-17*d, -(final_boundary-17*d), -(final_boundary-17*d), final_boundary-18*d,    final_boundary-18*d, -(final_boundary-18*d), -(final_boundary-18*d), final_boundary-19*d,     final_boundary-19*d, -(final_boundary-19*d), -(final_boundary-19*d), final_boundary-20*d};

    std::stack<geometry_msgs::Pose2D> stack1_prelim, stack2_prelim, stack3_prelim;
    std::stack<geometry_msgs::Pose2D> stack1_final, stack2_final, stack3_final;
};

#endif /* SEARCH_CONTROLLER */
