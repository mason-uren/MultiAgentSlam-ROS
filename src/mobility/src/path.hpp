#ifndef PATH_HPP
#define PATH_HPP

#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose2D.h>

namespace csuci {
    struct path_node {
        geometry_msgs::Pose2D pose;
        double time_offset;
        int indx;

        path_node* next;
    };

    struct Path {

    };
}

#endif //PATH_HPP
