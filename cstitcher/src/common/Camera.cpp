#include "common/Camera.h"

namespace cstitcher {
namespace common {

Camera::Camera() {
}

Camera::~Camera() {
}

void Camera::load(std::string bgrIntrinsicFile, std::string depthIntrinsicFile, std::string extrinsicFile) {
    //load intrinsics for RGB sensor
    cv::FileStorage fs(bgrIntrinsicFile, cv::FileStorage::READ);
    CV_Assert(fs.isOpened());
    cv::FileNode fn = fs["camera"];
    fn["K"] >> bgrCamera;
    fs["D"] >> bgrDistortion;
    fn["width"] >> bgrSize.width;
    fn["height"] >> bgrSize.height;
    CV_Assert(bgrSize.width != 0 || bgrSize.height != 0);
    fs.release();

    //load intrinsics for Depth sensor
    fs.open(depthIntrinsicFile, cv::FileStorage::READ);
    CV_Assert(fs.isOpened());
    fn = fs["camera"];
    fn["K"] >> depthCamera;
    fn["D"] >> depthDistortion;
    fn["width"] >> depthSize.width;
    fn["height"] >> depthSize.height;
    CV_Assert(depthSize.width != 0 || depthSize.height != 0);
    fs.release();

    //load extrinsics for RGB<->Depth sensors
    fs.open(extrinsicFile, cv::FileStorage::READ);
    CV_Assert(fs.isOpened());
    cv::Mat r, t;
    fs["R"] >> r;
    fs["T"] >> t;
    CV_Assert( !(r.empty()) || !(t.empty()));
    extrinsicR = cv::Matx33d(r);
    extrinsicT = cv::Vec3d(t);
}

}
}
