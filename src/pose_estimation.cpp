#include "pose_estimation.hpp"
#include <Eigen/Geometry>
#include <apriltag/apriltag_pose.h>
#include <apriltag/common/homography.h>
#include <opencv2/calib3d.hpp>
#include <tf2/convert.h>
#include <cmath>

// Distance correction function
double correctDistance(double measured_distance)
{
    double x = measured_distance;
    // return x;
    return 0.1998 * x * x * x - 0.1272 * x * x + 1.2450 * x + 0.0092;
    
}


geometry_msgs::msg::Transform
homography(apriltag_detection_t* const detection, const std::array<double, 4>& intr, double tagsize)
{
    apriltag_detection_info_t info = {detection, tagsize, intr[0], intr[1], intr[2], intr[3]};

    apriltag_pose_t pose;
    estimate_pose_for_tag_homography(&info, &pose);

    // rotate frame such that z points in the opposite direction towards the camera
    for(int i = 0; i < 3; i++) {
        // swap x and y axes
        std::swap(MATD_EL(pose.R, 0, i), MATD_EL(pose.R, 1, i));
        // invert z axis
        MATD_EL(pose.R, 2, i) *= -1;
    }

    auto transform = tf2::toMsg<apriltag_pose_t, geometry_msgs::msg::Transform>(const_cast<const apriltag_pose_t&>(pose));
    
    // Apply distance correction
    double measured_distance = std::sqrt(
        // transform.translation.x * transform.translation.x +
        // transform.translation.y * transform.translation.y +
        transform.translation.z * transform.translation.z
    );
    
    double corrected_distance = correctDistance(measured_distance);
    
    // Scale translation vector to corrected distance
    if (measured_distance > 0) {
        double scale_factor = corrected_distance / measured_distance;
        // transform.translation.x *= scale_factor;
        // transform.translation.y *= scale_factor;
        transform.translation.z *= scale_factor;
    }
    
    return transform;
}

geometry_msgs::msg::Transform
pnp(apriltag_detection_t* const detection, const std::array<double, 4>& intr, double tagsize)
{
    const std::vector<cv::Point3d> objectPoints{
        {-tagsize / 2, -tagsize / 2, 0},
        {+tagsize / 2, -tagsize / 2, 0},
        {+tagsize / 2, +tagsize / 2, 0},
        {-tagsize / 2, +tagsize / 2, 0},
    };

    const std::vector<cv::Point2d> imagePoints{
        {detection->p[0][0], detection->p[0][1]},
        {detection->p[1][0], detection->p[1][1]},
        {detection->p[2][0], detection->p[2][1]},
        {detection->p[3][0], detection->p[3][1]},
    };

    cv::Matx33d cameraMatrix;
    cameraMatrix(0, 0) = intr[0];// fx
    cameraMatrix(1, 1) = intr[1];// fy
    cameraMatrix(0, 2) = intr[2];// cx
    cameraMatrix(1, 2) = intr[3];// cy

    cv::Mat rvec, tvec;
    cv::solvePnP(objectPoints, imagePoints, cameraMatrix, {}, rvec, tvec);

    auto transform = tf2::toMsg<std::pair<cv::Mat_<double>, cv::Mat_<double>>, geometry_msgs::msg::Transform>(std::make_pair(tvec, rvec));
    
    // Apply distance correction
    double measured_distance = std::sqrt(
        // transform.translation.x * transform.translation.x +
        // transform.translation.y * transform.translation.y +
        transform.translation.z * transform.translation.z
    );
    
    double corrected_distance = correctDistance(measured_distance);
    
    // Scale translation vector to corrected distance
    if (measured_distance > 0) {
        double scale_factor = corrected_distance / measured_distance;
        // transform.translation.x *= scale_factor;
        // transform.translation.y *= scale_factor;
        transform.translation.z *= scale_factor;
    }
    
    return transform;
}

const std::unordered_map<std::string, pose_estimation_f> pose_estimation_methods{
    {"homography", homography},
    {"pnp", pnp},
};
