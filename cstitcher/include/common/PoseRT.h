#ifndef __POSE_RT_H__
#define __POSE_RT_H__

#include <opencv2/core/core.hpp>
#include <string>

namespace cstitcher {
namespace common {

class PoseRT {
public:
    PoseRT();
    PoseRT(cv::Vec4d quaternion, cv::Vec3d translation);
    ~PoseRT();

    void load(const std::string& filename);
    void load_res(const std::string& filename);
    void setRT(cv::Vec4d quaternion, cv::Vec3d translation);

    PoseRT inv();

    cv::Matx33d getR() const;
    cv::Vec3d getT() const;
    void setT(cv::Vec3d t);
    cv::Matx44d getPose() const;
    void setPose(cv::Matx44d pose_);
protected:
    cv::Matx33d R;
    cv::Vec3d t;
    cv::Matx44d pose;
    cv::Matx33d convertQuaternionToRotationMatrix(cv::Vec4d quat);
    void buildPose();
};

}
}

inline std::ostream& operator <<(std::ostream& out, const cstitcher::common::PoseRT& pose) {
    out <<"\n================\n"<< "R = " << pose.getR()<<"\n T = "<<pose.getT() <<"\n================\n";
    return out;
}

inline cstitcher::common::PoseRT operator *(const cstitcher::common::PoseRT& a, const cstitcher::common::PoseRT& b) {
    cstitcher::common::PoseRT tmp;
    tmp.setPose(a.getPose() * b.getPose());
    return tmp;
}

#endif /*__POSE_RT_H__*/
