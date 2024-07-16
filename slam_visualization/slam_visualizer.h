#ifndef SLAM_VISUALIZER_H_
#define SLAM_VISUALIZER_H_

#include <iostream>
#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "custom_struct.h"

using namespace std;

typedef Eigen::Matrix<float, 7, 1> BBox;

class SlamVisualizer
{
public:
    SlamVisualizer(int width = 752, int height = 480):WIN_WIDTH_(width), WIN_HEIGHT_(height){}
    ~SlamVisualizer(){}

    void initDraw();

    void activeAllView();

    void drawCubeTest();

    void drawBox(BBox& box);

    void drawCamWithPose(Eigen::Vector3d &pos, Eigen::Quaterniond &quat);

    void drawTraj(vector<Eigen::Vector3d> &traj);
    /**
     * @brief 画一个简单的相机模型
     * 
     * @param scale：缩放尺寸，默认为1 
     */
    void drawCam(const float scale = 1.);

    void drawCoordinate();

    void drawKeyFrame(Eigen::Quaterniond& quat);

    void displayImg(cv::Mat &originImg, cv::Mat& trackImg);

    void displayPcs(vector<Eigen::Vector3d>& pcs, Eigen::Vector3d& pos, Eigen::Quaterniond& quat, 
        vector<vector<Eigen::Vector3d>>& keyframe_pcs, vector <vector<Eigen::Vector3d>>& totalframe_pcs);

    void displayData(Eigen::Vector3d &pos, Eigen::Quaterniond &quat);

    void registerUICallback();

private:
    pangolin::OpenGlRenderState s_cam_;
    pangolin::View d_cam_, d_img_, d_track_;
    pangolin::GlTexture imageTexture_, trackTexture_;
    pangolin::DataLog pose_log_;

    // 存储ui面板的控件对象
    std::vector<pangolin::Var<bool>> ui_set_;
    // 存储data面板的控件对象
    std::vector<pangolin::Var<VecXd>> data_set_;
    // 上一帧pose
    Eigen::Vector3d pos_last_;
    // 关键帧数据
    std::vector<Eigen::Matrix4f> key_frames_;
    // 是否显示相机
    bool camera_visible_ = true;
    // 是否显示轨迹
    bool traj_visible_ = true;
    // 是否显示参考坐标系
    bool coordinate_visible_ = true;
    // 是否显示图像
    bool img_visible_ = true;
    // 是否显示点云
    bool pc_visible_ = true;
    // 是否显示关键帧
    bool keyframe_visible_ = true;
    // 是否为初始帧
    bool initialized = false;
    // 关键帧最小间隔
    int debounce_count = 0;
    // 窗口尺寸
    int WIN_WIDTH_;
    int WIN_HEIGHT_;
};

#endif