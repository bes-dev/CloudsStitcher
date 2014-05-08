#include "gtest/gtest.h"
#include "common/Camera.h"
#include <iostream>

using namespace cstitcher::common;

TEST(Camera, loadValidData) {
    Camera camera;
    camera.load("intrinsics_ios_vga.yml", "intrinsics_ss.yml", "extrinsics.yml");
}
