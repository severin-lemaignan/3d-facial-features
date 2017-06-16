
#include <string>
#include <ros/ros.h>

#include "facialfeaturescloud.hpp"

using namespace std;

int main(int argc, char* argv[])
{
    //ROS initialization
    ros::init(argc, argv, "face_features_3d");
    ros::NodeHandle rosNode;
    ros::NodeHandle _private_node("~");
    //
    // load parameters
    string modelFilename;
    _private_node.param<string>("face_model", modelFilename, "");

    if (modelFilename.empty()) {
        ROS_ERROR_STREAM("You must provide the face model with the parameter face_model.\n" <<
                         "For instance, _face_model:=shape_predictor_68_face_landmarks.dat");
        return(1);
    }

    FacialFeaturesPointCloudPublisher facialfeaturespublisher(rosNode, modelFilename);



    ROS_INFO("3d_face_features is ready. Waiting for pair of {rgb, depth} images!");
    ros::spin();

    return 0;
}

