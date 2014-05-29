#include "common/PoseRT.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>

namespace cstitcher {
namespace common {

PoseRT::PoseRT() {
    R = cv::Matx33d::all(0);
    t = cv::Vec3d::all(0);
}

PoseRT::PoseRT(cv::Vec4d quaternion, cv::Vec3d translation) {
    setRT(quaternion, translation);
}

PoseRT::~PoseRT(){}

cv::Matx33d PoseRT::convertQuaternionToRotationMatrix(cv::Vec4d quat) {
    double theta = 2 * acos(quat[0]);
    cv::Vec3d rvec = cv::Vec3d(quat[1], quat[2], quat[3]);
    double norm = sin(theta / 2);
    if(fabs(norm) > 1e-7){
        rvec *= theta / norm;
    } else {
        rvec *= 0.0;
    }
    cv::Matx33d rmat;
    cv::Rodrigues(cv::Mat(rvec), rmat);
    return rmat;
}

void PoseRT::setRT(cv::Vec4d quaternion, cv::Vec3d translation) {
    R = convertQuaternionToRotationMatrix(quaternion);
    t = cv::Mat(translation);
    t = -R*t;
    buildPose();
}

void PoseRT::buildPose() {
    pose(0, 0) = R(0, 0); pose(0, 1) = R(0, 1); pose(0, 2) = R(0, 2); pose(0, 3) = t[0];
    pose(1, 0) = R(1, 0); pose(1, 1) = R(1, 1); pose(1, 2) = R(1, 2); pose(1, 3) = t[1];
    pose(2, 0) = R(2, 0); pose(2, 1) = R(2, 1); pose(2, 2) = R(2, 2); pose(2, 3) = t[2];
    pose(3, 0) = 0.0f;    pose(3, 1) = 0.0f;    pose(3, 2) = 0.0f;    pose(3, 3) = 1.0f;
}

cv::Matx44d PoseRT::getPose() const {
    return pose;
}

cv::Matx33d PoseRT::getR() const {
    return R;
}

cv::Vec3d PoseRT::getT() const {
    return t;
}
    
void PoseRT::setT(cv::Vec3d _t)
{
    t = _t;
    buildPose();
}

void PoseRT::load(const std::string& filename) {
    cv::FileStorage fs;
    fs.open(filename, cv::FileStorage::READ);
    cv::Mat rotation, translation;
    fs["R"] >> rotation;
    fs["T"] >> translation;
    cv::Vec4d rvec = cv::Vec4d(rotation);
    cv::Vec3d tvec = cv::Vec3d(translation);
    setRT(rvec, tvec);
}

void PoseRT::load_res(const std::string& filename) {
    cv::FileStorage fs;
    fs.open(filename, cv::FileStorage::READ);
    cv::Mat poseFile;
    fs["pose"] >> poseFile;
    pose = poseFile;
    
    setPose(pose);
}

void PoseRT::setPose(cv::Matx44d pose_) {
    pose = pose_;
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            if(std::abs(pose(i, j)) < 1e-7) {
                pose(i, j) = 0.0f;
            }
        }
    }
    R(0, 0) = pose(0, 0); R(0, 1) = pose(0, 1); R(0, 2) = pose(0, 2); t[0] = pose(0, 3);
    R(1, 0) = pose(1, 0); R(1, 1) = pose(1, 1); R(1, 2) = pose(1, 2); t[1] = pose(1, 3);
    R(2, 0) = pose(2, 0); R(2, 1) = pose(2, 1); R(2, 2) = pose(2, 2); t[2] = pose(2, 3);
}

PoseRT PoseRT::inv() {
    PoseRT invPose;
    invPose.setPose(pose.inv());
    return invPose;
}

}
}
