#include <iostream>
#include <fstream>
#include <string>

#include "common/PointCloud.h"
#include "common/Camera.h"

#define POSE( x ) ((std::ostringstream()<<"pose_"<< x <<".yml")).str()
#define POSE_SBA( x ) ((std::ostringstream()<<"pose_sba_"<< x <<".xml")).str()
#define DEPTH( x ) ((std::ostringstream()<<"ios_depth_vga_"<< x <<".png")).str()
#define BGR_IMG( x ) ((std::ostringstream()<<"ios_image_vga_"<< x <<".png")).str()
#define PLY( x ) ((std::ostringstream()<<"pose_"<< x <<".ply")).str()


using namespace cstitcher::common;

int main(int argc, char* argv[]) {
    Camera camera;
    camera.load("intrinsics_ios_vga.yml", "intrinsics_ss.yml", "extrinsics.yml");
    std::vector<PointCloud> clouds;
    std::ifstream is;
    is.open("clouds.list");
    std::string s;
    PoseRT invFirst;
    int count = 0;
    std::cout<<"start load base\n";
    while(std::istream::sentry(is)) {
        is >> s;
        PointCloud cloud;
        cloud.open(DEPTH(s), BGR_IMG(s), camera);
        cloud.save(PLY(s));
        PoseRT pose;
        pose.load(POSE(s));
        if(!count) {
            invFirst = pose.inv();
            count++;
        }
        pose = pose * invFirst;
        pose = pose.inv();
        
        cv::Vec3d tr = pose.getT();
        const float median_scale = 0.19665410692;
        tr = tr*median_scale;
        pose.setT(tr);
        
        std::cout<<pose;
//
//        PoseRT pose2;
//        pose2.load_res(POSE_SBA(s));
//        std::cout<<pose2;
        
        cloud.applyPose(pose);
        clouds.push_back(cloud);
        std::cout<<s<<std::endl;
    }
    std::cout<<"end load base\n";
    std::cout<<"start save base\n";
    int size = 0;
    for(int i = 0; i < clouds.size(); i++) {
        size += clouds[i].size();
    }
    std::cout<<"size = "<<size<<std::endl;
    std::ofstream os;
    os.open("out.ply");
    os<<"ply\n";
    os<<"format ascii 1.0\n";
    os<<"comment : created from Kinect depth image\n";
    os<<"element vertex "<<size<<"\n";
    os<<"property float x\n";
    os<<"property float y\n";
    os<<"property float z\n";
    os<<"property uchar blue\n";
    os<<"property uchar green\n";
    os<<"property uchar red\n";
    os<<"end_header\n";
    for(int i = 0; i < clouds.size(); i++) {
        std::cout<<i<<std::endl;
        std::vector<cv::Vec3d> points;
        std::vector<cv::Vec3i> colors;
        clouds[i].getCloud(points, colors);
        for(int j = 0; j < points.size(); j++) {
            os<<points[j][0]<<" "<<points[j][1]<<" "<<points[j][2]<<" "<<colors[j][0]<<" "<<colors[j][1]<<" "<<colors[j][2]<<"\n";
        }
    }
    os.close();
    std::cout<<"end save base\n";
    return 0;
}
