#include <pangolin/pangolin.h>
#include <queue>
#include <stdio.h>
//#include <unistd.h>
#include "slam_visualizer.h"
#include <thread>
#include <random>

using namespace std;

SlamVisualizer visualizer(1504, 960);
queue<string> imgFileNames;
queue<long> imgTimeStamps;

vector<Eigen::Vector3d> generateRandomVectors(vector<Eigen::Vector3d>& vectors, size_t numVectors, double minX = -1.0, double maxX = 1.0) {
    vectors.clear();
    vectors.reserve(numVectors);

    // 使用C++11的随机数库  
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<> dis(minX, maxX);

    for (size_t i = 0; i < numVectors; ++i) {
        Eigen::Vector3d vec;
        vec(0) = dis(gen); // 随机X  
        vec(1) = dis(gen); // 随机Y  
        vec(2) = dis(gen); // 随机Z  
        vectors.push_back(vec);
    }

    return vectors;
}


int main(int argc, char** argv){
    FILE *fp_gt, *fp_img;
    // 请自行修改数据集路径
    fp_gt = fopen("/home/hosico/DataDisk/hdd2/Dataset/euroc/mav0/state_groundtruth_estimate0/data.csv", "r");
    fp_img = fopen("/home/hosico/DataDisk/hdd2/Dataset/euroc/mav0/cam0/data.csv", "r");
    if(fp_gt == nullptr || fp_img == nullptr){
        cout << "failed to open file !\n";
        return -1;
    }
    // =================== 读取图片路径 ====================//
     // 跳过第一行
    char fl_buf[1024];
    fgets(fl_buf, sizeof(fl_buf), fp_img);
  
    while(!feof(fp_img)){
        char filename[23];
        long timestamp;
        fscanf(fp_img, "%lu,%s", &timestamp, filename);
        
        imgTimeStamps.push(timestamp);
        imgFileNames.push(string(filename));
    }
    // ===================读取groundtruth =================================== //
    // 跳过第一行
    fgets(fl_buf, sizeof(fl_buf), fp_gt);

    // 初始化视窗
    visualizer.initDraw();
    vector<Eigen::Vector3d> traj;
    vector<Eigen::Vector3d> pcs;
    vector<vector<Eigen::Vector3d>> keyframe_pcs;
    vector <vector<Eigen::Vector3d>> totalframe_pcs;

    int numVectors = 5;

    while (!feof(fp_gt))
    {
        // 常规操作
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
        visualizer.activeAllView();
        // 注册ui回调函数
        visualizer.registerUICallback();
        // 从数据集中读取数据
        // 创建数据寄存器    
        long time_stamp(0);
        double px(0.), py(0.), pz(0.);
        double qw(0.), qx(0.), qy(0.), qz(0.);
        double vx(0.), vy(0.), vz(0.);
        double bwx(0.), bwy(0.), bwz(0.), bax(0.), bay(0.), baz(0.);
        fscanf(fp_gt, "%lu,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
            &time_stamp, &px, &py, &pz,
            &qw, &qx, &qy, &qz,
            &vx, &vy, &vz,
            &bwx, &bwy, &bwz,
            &bax, &bay, &baz);

        Eigen::Quaterniond quat(qw, qx, qy, qz); //quat是否要转置？
        Eigen::Vector3d pos(px, py, pz);
        traj.push_back(pos);
        pcs = generateRandomVectors(pcs, numVectors);

        // add box
        BBox bbox, bbox1;
        bbox << 3.0f, 7.0f, 0.5f, 2.0f, 1.0f, 1.0f, 0.0f;
        bbox1 << 1.0f, 2.0f, 3.0f, 1.1f, 2.1f, 2.0f, 45.0f;
        vector<BBox> boxes;
        boxes.push_back(bbox);
        boxes.push_back(bbox1);
        // 显示数据
        for (auto& box : boxes) {
            visualizer.drawBox(box);
        }
        visualizer.displayData(pos, quat);
        visualizer.displayPcs(pcs, pos, quat, keyframe_pcs, totalframe_pcs);
        // 绘制轨迹可视化部分
        visualizer.drawCoordinate();
        visualizer.drawCamWithPose(pos, quat);
        visualizer.drawTraj(traj);
        // 弹出当前时刻之前的图像
        double imu_time, img_time;
        imu_time = (double)time_stamp / 1e9; 
        img_time = (double)imgTimeStamps.front() / 1e9;

        if(imu_time > img_time){
            // cout << imgFileNames.front() << endl;
            imgTimeStamps.pop();
            imgFileNames.pop();
        }
        // cout << "current size of img: " << imgFileNames.size() << endl;
        // 显示图像(由于数据集没有跟踪图像，这里两幅图像显示一样)
        cv::Mat img;
        string img_file = "/home/hosico/DataDisk/hdd2/Dataset/euroc/mav0/cam0/data/" + imgFileNames.front();
        img = cv::imread(img_file, CV_LOAD_IMAGE_COLOR);
        visualizer.drawKeyFrame(quat);
        visualizer.displayImg(img, img);
        // 循环与退出判断
        pangolin::FinishFrame();
        
        if(pangolin::ShouldQuit())
            break;

        // 200hz
        // usleep(5000); // 挂起5ms
    }

    return 0;
}

