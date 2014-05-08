#ifndef __CAMERA_H__
#define __CAMERA_H__

#include <string>
#include <opencv2/core/core.hpp>

namespace cstitcher {
namespace common {

class Camera {
public:
    Camera();
    ~Camera();

    void load(std::string bgrIntrinsicFile, std::string depthIntrinsicFile, std::string extrinsicFile);

    cv::Size bgrSize;
    cv::Mat bgrCamera;
    cv::Mat bgrDistortion;

    cv::Size depthSize;
    cv::Mat depthCamera;
    cv::Mat depthDistortion;

    cv::Matx33d extrinsicR;
    cv::Vec3d extrinsicT;
};

}
}

#endif /*__CAMERA_H__*/
