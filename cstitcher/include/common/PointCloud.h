#ifndef __POINT_CLOUD_H__
#define __POINT_CLOUD_H__

#include <vector>
#include <opencv2/core/core.hpp>
#include "common/Camera.h"
#include "common/PoseRT.h"

namespace cstitcher {
namespace common {

class PointCloud {
public:
    PointCloud();
    ~PointCloud();

    void open(std::string depthFile, std::string bgrFile, Camera& camera);
    void applyPose(const PoseRT& pose);
    void getCloud(std::vector<cv::Vec3d>& points, std::vector<cv::Vec3i>& colors);
    int size();

    void save(const std::string& fileName);
protected:
    std::vector<cv::Vec3d> points;
    std::vector<cv::Vec3i> colors;
};

}
}

#endif /*__POINT_CLOUD_H__*/
