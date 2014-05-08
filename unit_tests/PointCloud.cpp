#include "gtest/gtest.h"
#include "common/PointCloud.h"
#include "common/Camera.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <fstream>

using namespace cstitcher::common;

TEST(PointCloud, open) {
    Camera camera;
    camera.load("intrinsics_ios_vga.yml", "intrinsics_ss.yml", "extrinsics.yml");
    PointCloud cloud;
    cloud.open("ios_depth_vga_00030.png", "ios_image_vga_00030.png", camera);
    cloud.save("out.ply");
}
