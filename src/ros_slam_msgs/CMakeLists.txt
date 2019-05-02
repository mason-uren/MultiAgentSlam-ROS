cmake_minimum_required(VERSION 2.8.3)
project(ros_slam_msgs)

find_package(catkin REQUIRED COMPONENTS
        message_generation
        std_msgs
)

add_message_files(
        FILES
            AgentPairs.msg
            AuxBeliefs.msg
            AuxFeatureSet.msg
            Belief.msg
            Classifier.msg
            Feature.msg
            FeatureSet.msg
            Offset.msg
            Pose.msg
            TransformationPairs.msg
            Transformation.msg
)

generate_messages(
        DEPENDENCIES
            std_msgs
)

catkin_package(
        INCLUDE_DIRS
            msg
        CATKIN_DEPENDS
            message_runtime
)
