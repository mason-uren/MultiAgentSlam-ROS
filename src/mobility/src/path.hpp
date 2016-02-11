#ifndef PATH_HPP
#define PATH_HPP

#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose2D.h>

namespace csuci {

    const double DEFAULT_VELOCITY = 0.3;
    const double DEFAULT_ANG_VELOCITY = 0.2;

    class PathNode;
    class Path;

    class PathNode {
        public:
            PathNode(double curr_x, double curr_y, double curr_theta, double x, double y);
            PathNode(PathNode* prev, double x, double y);
            PathNode(PathNode* prev, PathNode* next, double x, double y);
            PathNode(const PathNode& pn);
            ~PathNode();

            PathNode& operator=(const PathNode& pn);

            PathNode* Next();
            PathNode* Prev();

            const geometry_msgs::Pose2D& Goal() const;
            double TravelTime() const;
            int Index() const;

        private:
            geometry_msgs::Pose2D goal_pose;
            double travel_time;
            int indx;

            PathNode* next;
            PathNode* prev;

            friend class Path;
    };

    class Path {
        public:
            Path();
            ~Path();

            void Insert(size_t idx, double curr_x, double curr_y, double curr_theta, double x, double y);
            PathNode* Get(size_t idx);
            void Remove(size_t idx);

        private:
            PathNode* head;
            PathNode* tail;
            size_t size;
    };
}

#endif //PATH_HPP
