#include <string>

// opencv3
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>

#include <image_geometry/pinhole_camera_model.h>

#include "head_pose_estimation.hpp"

using namespace std;
using namespace cv;

image_geometry::PinholeCameraModel rgbcameramodel;
image_geometry::PinholeCameraModel depthcameramodel;
cv::Mat cameraMatrix, distCoeffs;
bool warnUncalibratedImage = true;

cv::Mat inputImage;
HeadPoseEstimation* estimator;

void process(const sensor_msgs::ImageConstPtr& rgb_msg,
             const sensor_msgs::CameraInfoConstPtr& rgb_camerainfo,
             const sensor_msgs::ImageConstPtr& depth_msg,
             const sensor_msgs::CameraInfoConstPtr& depth_camerainfo) {

    ROS_INFO_ONCE("First image received");

    // updating the camera model is cheap if not modified
    rgbcameramodel.fromCameraInfo(rgb_camerainfo);
    depthcameramodel.fromCameraInfo(depth_camerainfo);
    // publishing uncalibrated images? -> return (according to CameraInfo message documentation,
    // K[0] == 0.0 <=> uncalibrated).
    if(rgbcameramodel.intrinsicMatrix()(0,0) == 0.0) {
        if(warnUncalibratedImage) {
            warnUncalibratedImage = false;
            ROS_ERROR("Camera publishes uncalibrated images. Can not estimate face position.");
            ROS_WARN("Detection will start over again when camera info is available.");
        }
        return;
    }
    warnUncalibratedImage = true;

    auto rgb = cv_bridge::toCvShare(rgb_msg, "bgr8")->image; 
    auto depth = cv_bridge::toCvShare(depth_msg)->image; 

    // hopefully no copy here:
    //  - assignement operator of cv::Mat does not copy the data
    //  - toCvShare does no copy if the default (source) encoding is used.
    Mat inputImage = cv_bridge::toCvShare(rgb_msg)->image;

    // got an empty image!
    if (inputImage.size().area() == 0) return;

    /********************************************************************
    *                      Faces detection                           *
    ********************************************************************/

    estimator->update(inputImage);

    auto poses = estimator->poses();
    ROS_INFO_STREAM(poses.size() << " faces detected.");

    //uint16_t *image_data = (uint16_t *) depth.data;

    //float g_depth_avg;
    //double depth_total = 0;
    //int depth_count = 0;
    //for (unsigned int i = 0; i < depth_msg->height * depth_msg->width; ++i)
    //{
    //    //if ((0 < *image_data) && (*image_data <= g_max_z))
    //    if ((0 < *image_data))
    //    {
    //        depth_total += *image_data;
    //        depth_count++;
    //    }
    //    image_data++;
    //}
    //if (depth_count != 0)
    //{
    //    g_depth_avg = static_cast<float>(depth_total / depth_count);
    //}

    //ROS_INFO_STREAM("Avg depth: " << g_depth_avg);

    depth.convertTo(depth, CV_32F); // thresholding works on CV_8U or CV_32F but not CV_16U
    imshow("Input depth", depth);
    //threshold(depth, depth, 0.5, 1.0, THRESH_BINARY_INV);
    //depth.convertTo(depth, CV_8U); // masking requires CV_8U. All non-zero values are kept, so '1.0' is fine

    //Mat maskedImage;
    //rgb.copyTo(maskedImage, depth);

    //imshow("Input RGB", rgb);
    imshow("2D face features", estimator->_debug);
    //imshow("Masked input", maskedImage);
    waitKey(10);
}

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

    estimator = new HeadPoseEstimation(modelFilename);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(rosNode, "rgb", 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> rgb_camera_info_sub(rosNode, "rgb_camera_info", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(rosNode, "depth", 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> depth_camera_info_sub(rosNode, "depth_camera_info", 1);

    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo> sync(rgb_sub, rgb_camera_info_sub, depth_sub, depth_camera_info_sub, 9);
    sync.registerCallback(bind(&process, _1, _2, _3, _4 ) );

    ROS_INFO("3d_face_features is ready. Waiting for pair of {rgb, depth} images!");
    ros::spin();

    return 0;
}

