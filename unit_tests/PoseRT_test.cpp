#include "gtest/gtest.h"
#include "common/PoseRT.h"
#include <iostream>

using namespace cstitcher::common;

TEST(PoseRT, initWithValidRt) {
    cv::Vec4f q(0, 0, 0, 1);
    cv::Vec3f t(1, 1, 1);

    PoseRT pose(q, t);
    cv::Vec3d v(1, 1, 1);
    v = pose.getR() * v;
    v = v + pose.getT();

    ASSERT_EQ(v, cv::Vec3d(2, 2, 2));
}

//TEST(PoseRT, validInvPose) {
//    PoseRT pose, invPose;
//    cv::Vec4f q(1, 0.5, 0, 1);
//    cv::Vec3f t(1, 1, 1);
//    pose.setRT(q, t);
//    invPose = pose.inv();
//    cv::Matx44d m;
//    m = pose.getPose() * invPose.getPose();
//    cv::Matx44d identity;
//    identity(0, 0) = 1; identity(0, 1) = 0; identity(0, 2) = 0; identity(0, 3) = 0;
//    identity(1, 0) = 0; identity(1, 1) = 1; identity(1, 2) = 0; identity(1, 3) = 0;
//    identity(2, 0) = 0; identity(2, 1) = 0; identity(2, 2) = 1; identity(2, 3) = 0;
//    identity(3, 0) = 0; identity(3, 1) = 0; identity(3, 2) = 0; identity(3, 3) = 1;

//    ASSERT_TRUE(m == identity);
//}

//TEST(PoseRT, validMultOperator) {
//    PoseRT pose, invPose;
//    cv::Vec4f q(0.956407733138, 0.290833663953, -0.00748394296084, 0.0253847637967);
//    cv::Vec3f t(1, 1, 1);
//    pose.setRT(q, t);
//    invPose = pose.inv();
//    PoseRT m = pose * invPose;
//    cv::Matx44d identity;
//    identity(0, 0) = 1; identity(0, 1) = 0; identity(0, 2) = 0; identity(0, 3) = 0;
//    identity(1, 0) = 0; identity(1, 1) = 1; identity(1, 2) = 0; identity(1, 3) = 0;
//    identity(2, 0) = 0; identity(2, 1) = 0; identity(2, 2) = 1; identity(2, 3) = 0;
//    identity(3, 0) = 0; identity(3, 1) = 0; identity(3, 2) = 0; identity(3, 3) = 1;

//    cv::Matx33d identityR;
//    identityR(0, 0) = 1; identityR(0, 1) = 0; identityR(0, 2) = 0;
//    identityR(1, 0) = 0; identityR(1, 1) = 1; identityR(1, 2) = 0;
//    identityR(2, 0) = 0; identityR(2, 1) = 0; identityR(2, 2) = 1;
//    cv::Vec3d identityT(0.0f, 0.0f, 0.0f);

//    ASSERT_TRUE(m.getPose() == identity);
//    ASSERT_TRUE(m.getR() == identityR);
//    ASSERT_TRUE(m.getT() == identityT);
//}
