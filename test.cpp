///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2020, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

/***********************************************************************************************
 ** This sample demonstrates how to reloc a ZED camera using an ArUco marker.                  **
 ** Images are captured with the ZED SDK and cameras poses is then computed from ArUco pattern **
 ** to reset ZED tracking with this known position.                                            **
 ***********************************************************************************************/

// ZED includes
#include <sl/Camera.hpp>

// Sample includes
#include "aruco.hpp"

// OCV includes
#include <opencv2/opencv.hpp>

// ROS import
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "detect_aruco/Num.h"

using namespace sl;
using namespace std;

int main(int argc, char **argv)
{
    // new code
    ros::init(argc, argv, "aruco_detector");
    ros::NodeHandle n;
    ros::Publisher aruco_pub = n.advertise<detect_aruco::Num>("aruco_info", 100);
    ros::Rate loop_rate(60);
    // new code end

    // Create a ZED camera object
    Camera zed;

    // Set configuration parameters
    InitParameters init_params;
    init_params.camera_resolution = RESOLUTION::HD2K;
    init_params.coordinate_units = UNIT::METER;
    init_params.sensors_required = false;

    // Open the camera
    ERROR_CODE err = zed.open(init_params);
    if (err != ERROR_CODE::SUCCESS)
    {
        cout << "Error, unable to open ZED camera: " << err << "\n";
        zed.close();
        return 1; // Quit if an error occurred
    }

    auto cameraInfo = zed.getCameraInformation().camera_configuration;

    Resolution image_size = cameraInfo.resolution;
    Mat image_zed(image_size, MAT_TYPE::U8_C4);
    cv::Mat image_ocv = cv::Mat(image_zed.getHeight(), image_zed.getWidth(), CV_8UC4, image_zed.getPtr<sl::uchar1>(MEM::CPU));
    cv::Mat image_ocv_rgb;

    auto calibInfo = cameraInfo.calibration_parameters.left_cam;
    cv::Matx33d camera_matrix = cv::Matx33d::eye();
    camera_matrix(0, 0) = calibInfo.fx;
    camera_matrix(1, 1) = calibInfo.fy;
    camera_matrix(0, 2) = calibInfo.cx;
    camera_matrix(1, 2) = calibInfo.cy;

    cv::Matx<float, 4, 1> dist_coeffs = cv::Vec4f::zeros();

    float actual_marker_size_meters = 0.20f; // real marker size in meters
    auto dictionary = aruco::getPredefinedDictionary(aruco::DICT_ARUCO_ORIGINAL);

    cout << "Make sure the ArUco marker is a 4X4_50, measuring " << actual_marker_size_meters * 1000 << " mm" << endl;

    Transform pose;
    Pose zed_pose;
    vector<cv::Vec3d> rvecs, tvecs;
    vector<int> ids;
    vector<vector<cv::Point2f>> corners;
    string position_txt;

    bool can_reset = false;

    PositionalTrackingParameters tracking_params;
    tracking_params.enable_imu_fusion = false; // for this sample, IMU (of ZED-M) is disable, we use the gravity given by the marker.
    zed.enablePositionalTracking(tracking_params);

    // new code
    int global_id = -1;
    float global_x = -1;
    float global_y = -1;
    float global_z = -1;

    // Calculate the center of the image frame
    float frame_center_x = image_ocv.cols / 2;
    float frame_center_y = image_ocv.rows / 2;
    // new code end

    // Loop until 'q' is pressed
    char key = '.';
    while (key != 'q')
    {
        // new code
        int local_id = -1;
        float local_x = -1;
        float local_y = -1;
        float local_z = -1;
        // new code end

        if (zed.grab() == ERROR_CODE::SUCCESS)
        {
            // Retrieve the left image
            zed.retrieveImage(image_zed, VIEW::LEFT, MEM::CPU, image_size);

            // convert to RGB
            cv::cvtColor(image_ocv, image_ocv_rgb, cv::COLOR_RGBA2RGB);
            // detect marker
            aruco::detectMarkers(image_ocv_rgb, dictionary, corners, ids);

            // get actual ZED position
            zed.getPosition(zed_pose);

            // display ZED position
            cv::rectangle(image_ocv_rgb, cv::Point(0, 0), cv::Point(490, 75), cv::Scalar(0, 0, 0), -1);
            cv::putText(image_ocv_rgb, "Loaded dictionary : 4X4.     Press 'SPACE' to reset the camera position", cv::Point(10, 15), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(220, 220, 220));
            position_txt = "ZED  x: " + to_string(zed_pose.pose_data.tx) + "; y: " + to_string(zed_pose.pose_data.ty) + "; z: " + to_string(zed_pose.pose_data.tz);
            cv::putText(image_ocv_rgb, position_txt, cv::Point(10, 35), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(236, 188, 26));

            // new code

            // cout << frame_center_x << "," << frame_center_y << "\n";
            // new code end

            // if at least one marker detected
            if (ids.size() > 0)
            {
                // std::string pubdt = "[";
                detect_aruco::Num msg;

                for (int i = 0; i < ids.size(); i++)
                {
                    aruco::estimatePoseSingleMarkers(corners, actual_marker_size_meters, camera_matrix, dist_coeffs, rvecs, tvecs);
                    pose.setTranslation(sl::float3(tvecs[i](0), tvecs[i](1), tvecs[i](2)));
                    pose.setRotationVector(sl::float3(rvecs[i](0), rvecs[i](1), rvecs[i](2)));
                    pose.inverse();
                    can_reset = true;

                    aruco::drawDetectedMarkers(image_ocv_rgb, corners, ids);
                    aruco::drawAxis(image_ocv_rgb, camera_matrix, dist_coeffs, rvecs[0], tvecs[0], actual_marker_size_meters * 0.5f);
                    position_txt = "Aruco x: " + to_string(pose.tx) + "; y: " + to_string(pose.ty) + "; z: " + to_string(pose.tz);
                    cv::putText(image_ocv_rgb, position_txt, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(124, 252, 124));

                    // new code

                    // code to determine wheather the tag is on the left or right
                    //  Calculate the center of the ArUco tag
                    cv::Point2f tag_center = cv::Point2f((corners[0][0].x + corners[0][2].x) / 2, (corners[0][0].y + corners[0][2].y) / 2);
                    // Transform the coordinate system so that the frame center becomes (0, 0)
                    cv::Point2f tag_center_relative(tag_center.x - frame_center_x, tag_center.y - frame_center_y);
                    // calculate the difference in x coordinate
                    float x_diff = tag_center_relative.x;

                    // Calculate the distance from the camera to the ArUco tag
                    float distance_to_aruco = sqrt(pose.tx * pose.tx + pose.ty * pose.ty + pose.tz * pose.tz);
                    // Print the distance to the ArUco tag
                    // cout << distance_to_aruco << " Meter\n\n";

                    int local_id = ids[i];
                    float local_x = float(pose.tx);
                    // Publish marker information

                    msg.total += to_string(local_id) + ",";
                    msg.total += to_string(x_diff) + ",";
                    msg.total += to_string(distance_to_aruco) + ",";

                    // new code end
                }
                msg.total.pop_back();
                aruco_pub.publish(msg);
            }
            else
                can_reset = false;

            // Display image
            cv::imshow("Image", image_ocv_rgb);

            // Handle key event
            key = cv::waitKey(10);

            // if KEY_R is pressed and aruco marker is visible, then reset ZED position
            if ((key == ' ') && can_reset)
                zed.resetPositionalTracking(pose);
        }
        // new code
        if (ros::ok())
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
        // new code end
    }
    zed.close();
    return 0;
}