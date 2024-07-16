#include "slam_visualizer.h"

inline float degreesToRadians(float degrees) {
    return degrees * M_PI / 180.0;
}

struct Point3D {
    GLfloat x, y, z;
    Point3D(GLfloat x, GLfloat y, GLfloat z) : x(x), y(y), z(z) {}
};

static void computeRotationMatrix(float angleRadians, float rotationMatrix[3][3]) {
    float c = cos(angleRadians);
    float s = sin(angleRadians);

    rotationMatrix[0][0] = c;
    rotationMatrix[0][1] = -s;
    rotationMatrix[0][2] = 0;

    rotationMatrix[1][0] = s;
    rotationMatrix[1][1] = c;
    rotationMatrix[1][2] = 0;

    rotationMatrix[2][0] = 0;
    rotationMatrix[2][1] = 0;
    rotationMatrix[2][2] = 1;
}

void SlamVisualizer::initDraw(){
    pangolin::CreateWindowAndBind("camera_pose", WIN_WIDTH_, WIN_HEIGHT_);
    glEnable(GL_DEPTH_TEST);

    s_cam_ = pangolin::OpenGlRenderState(
        pangolin::ProjectionMatrix(WIN_WIDTH_, WIN_HEIGHT_, 420, 420, 320, 240, 0.1, 1000),
        pangolin::ModelViewLookAt(5, -3, 5, 0, 0, 0, pangolin::AxisZ)
    );
    int PANEL_WIDTH = WIN_WIDTH_ / 4;
    int PANEL_HEIGHT = WIN_HEIGHT_ / 4;
    // 轨迹显示窗口
    d_cam_ = pangolin::CreateDisplay()
        .SetBounds(0., 1., pangolin::Attach::Pix(PANEL_WIDTH), 1., -(float)WIN_WIDTH_/ (float)WIN_HEIGHT_)
        .SetHandler(new pangolin::Handler3D(s_cam_));
    // 控制面板
    pangolin::CreatePanel("ui")
        .SetBounds(pangolin::Attach::Pix(3.0f *PANEL_HEIGHT), 1., 0., pangolin::Attach::Pix(PANEL_WIDTH), (float)WIN_WIDTH_/ (float)WIN_HEIGHT_);
    
    ui_set_.clear();
    pangolin::Var<bool> show_cam("ui.show_cam", true, true);
    ui_set_.push_back(show_cam); 
    pangolin::Var<bool> show_traj("ui.show_traj", true, true);
    ui_set_.push_back(show_traj); 
    pangolin::Var<bool> show_img("ui.show_img", true, true);
    ui_set_.push_back(show_img); 
    pangolin::Var<bool> show_pc("ui.show_pc", true, true);
    ui_set_.push_back(show_pc);
    pangolin::Var<bool> show_keyframe("ui.show_keyframe", true, true);
    ui_set_.push_back(show_keyframe);
    pangolin::Var<bool> show_coordinate("ui.show_coordinate", true, true);
    ui_set_.push_back(show_coordinate);
    pangolin::Var<bool> save_map("ui.save_map", false, false);
    ui_set_.push_back(save_map);
    pangolin::Var<bool> save_win("ui.save_win", false, false);
    ui_set_.push_back(save_win);

    // 数据显示
    pangolin::CreatePanel("data")
        .SetBounds(pangolin::Attach::Pix(2.0f *PANEL_HEIGHT), pangolin::Attach::Pix(3.0f *PANEL_HEIGHT),
                                    0., pangolin::Attach::Pix(PANEL_WIDTH), (float)WIN_WIDTH_/ (float)WIN_HEIGHT_);
    data_set_.clear();
    pangolin::Var<VecXd> curr_pos("data.pos", VecXd());
    data_set_.push_back(curr_pos);
    pangolin::Var<VecXd> curr_att("data.euler_angle", VecXd());
    data_set_.push_back(curr_att);
    // 原图片显示
    d_img_ = pangolin::CreateDisplay()
        .SetBounds(pangolin::Attach::Pix(1.0f *PANEL_HEIGHT), pangolin::Attach::Pix(2.0f *PANEL_HEIGHT), 
                0., pangolin::Attach::Pix(PANEL_WIDTH), (float)WIN_WIDTH_/ (float)WIN_HEIGHT_)
        .SetLock(pangolin::LockLeft, pangolin::LockBottom);

    ////imageTexture_(752, 480, GL_RGB, false, 0, GL_BGR, GL_UNSIGNED_BYTE);
    imageTexture_ = pangolin::GlTexture(752, 480, GL_RGB, false, 0, GL_BGR, GL_UNSIGNED_BYTE);
    //// 跟踪图片显示
    d_track_ = pangolin::CreateDisplay()
        .SetBounds(0., pangolin::Attach::Pix(1.0f *PANEL_HEIGHT), 
                0., pangolin::Attach::Pix(PANEL_WIDTH), (float)WIN_WIDTH_/ (float)WIN_HEIGHT_)
        .SetLock(pangolin::LockLeft, pangolin::LockBottom);

    ////trackTexture_(752, 480, GL_RGB, false, 0, GL_BGR, GL_UNSIGNED_BYTE);
    trackTexture_ = pangolin::GlTexture(752, 480, GL_RGB, false, 0, GL_BGR, GL_UNSIGNED_BYTE);
}

void SlamVisualizer::activeAllView(){
    d_cam_.Activate(s_cam_);
}

void SlamVisualizer::drawCubeTest(){
    // Render some stuff
    glColor3f(1.0,0.0,1.0);
    pangolin::glDrawColouredCube();
}

void SlamVisualizer::drawBox(BBox& box)
{
    // box to 8 points 
    // centerX, centerY, centerZ, Height, Width, Length, Heading
    float halfLength = box(3) / 2.0;
    float halfWidth = box(4) / 2.0;
    float halfHeight = box(5) / 2.0;

    // defline 8 points in center corrdinate 
    float localPoints[8][3] = {
        {-halfLength, -halfWidth, -halfHeight},
        { halfLength, -halfWidth, -halfHeight},
        { halfLength,  halfWidth, -halfHeight},
        {-halfLength,  halfWidth, -halfHeight},
        {-halfLength, -halfWidth,  halfHeight},
        { halfLength, -halfWidth,  halfHeight},
        { halfLength,  halfWidth,  halfHeight},
        {-halfLength,  halfWidth,  halfHeight}
    };

    float rotationMatrix[3][3];
    computeRotationMatrix(degreesToRadians(box(6)), rotationMatrix);

    vector<Point3D> corners;

    for (int i = 0; i < 8; ++i) {
        GLfloat x = box(0) + rotationMatrix[0][0] * localPoints[i][0] + rotationMatrix[0][1] * localPoints[i][1] + rotationMatrix[0][2] * localPoints[i][2];
        GLfloat y = box(1) + rotationMatrix[1][0] * localPoints[i][0] + rotationMatrix[1][1] * localPoints[i][1] + rotationMatrix[1][2] * localPoints[i][2];
        GLfloat z = box(2) + rotationMatrix[2][0] * localPoints[i][0] + rotationMatrix[2][1] * localPoints[i][1] + rotationMatrix[2][2] * localPoints[i][2];

        corners.push_back(Point3D(x, y, z));
    }

    const GLfloat verts[] = {
        // FRONT
        corners[0].x, corners[0].y, corners[0].z,  
        corners[1].x, corners[1].y, corners[1].z, 
        corners[2].x, corners[2].y, corners[2].z, 
        corners[3].x, corners[3].y, corners[3].z,
        // BACK
        corners[4].x, corners[4].y, corners[4].z,
        corners[5].x, corners[5].y, corners[5].z,
        corners[6].x, corners[6].y, corners[6].z,
        corners[7].x, corners[7].y, corners[7].z,
        // LEFT
        corners[0].x, corners[0].y, corners[0].z,
        corners[4].x, corners[4].y, corners[4].z,
        corners[7].x, corners[7].y, corners[7].z,
        corners[3].x, corners[3].y, corners[3].z,
        // RIGHT
        corners[1].x, corners[1].y, corners[1].z,
        corners[5].x, corners[5].y, corners[5].z,
        corners[6].x, corners[6].y, corners[6].z,
        corners[2].x, corners[2].y, corners[2].z,
        // TOP
        corners[3].x, corners[3].y, corners[3].z,
        corners[2].x, corners[2].y, corners[2].z,
        corners[6].x, corners[6].y, corners[6].z,
        corners[7].x, corners[7].y, corners[7].z,
        // BOTTOM
        corners[0].x, corners[0].y, corners[0].z,
        corners[1].x, corners[1].y, corners[1].z,
        corners[5].x, corners[5].y, corners[5].z,
        corners[4].x, corners[4].y, corners[4].z,
    };

    glVertexPointer(3, GL_FLOAT, 0, verts);
    glEnableClientState(GL_VERTEX_ARRAY);

    glColor4f(1.0f, 0.0f, 0.0f, 1.0f);  // R
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    glDrawArrays(GL_TRIANGLE_STRIP, 4, 4);

    glColor4f(0.0f, 1.0f, 0.0f, 1.0f); // G
    glDrawArrays(GL_TRIANGLE_STRIP, 8, 4);
    glDrawArrays(GL_TRIANGLE_STRIP, 12, 4);

    glColor4f(0.0f, 0.0f, 1.0f, 1.0f); // B
    glDrawArrays(GL_TRIANGLE_STRIP, 16, 4);
    glDrawArrays(GL_TRIANGLE_STRIP, 20, 4);

    glDisableClientState(GL_VERTEX_ARRAY);
}

void SlamVisualizer::drawCam(const float scale){

    if(scale < 0){
        cerr << "scale should be positive !\n";
        return;
    }
        
    const float w = 0.2 * scale;
    const float h = w * 0.75;
    const float z = w * 0.8;

    glLineWidth(2 * scale);
    // 绘制相机轮廓线
    glBegin(GL_LINES);
    glColor3f(0.0f,1.0f,1.0f);
	glVertex3f(0,0,0);		glVertex3f(w,h,z);
	glVertex3f(0,0,0);		glVertex3f(w,-h,z);
	glVertex3f(0,0,0);		glVertex3f(-w,-h,z);
	glVertex3f(0,0,0);		glVertex3f(-w,h,z);
	glVertex3f(w,h,z);		glVertex3f(w,-h,z);
	glVertex3f(-w,h,z);		glVertex3f(-w,-h,z);
	glVertex3f(-w,h,z);		glVertex3f(w,h,z);
	glVertex3f(-w,-h,z);    glVertex3f(w,-h,z);
	glEnd();

    return;
}

void SlamVisualizer::drawCamWithPose(Eigen::Vector3d &pos, Eigen::Quaterniond &quat){
    if (!camera_visible_)
        return;


    Eigen::Vector3d pos_delta_ = { 0, 0, 0 };

    Eigen::Matrix4f mat;
    Eigen::Matrix3d R = quat.toRotationMatrix();

    if (!initialized) {
        pos_last_ = pos;
        initialized = true;
    }     
    else {
        if (debounce_count) {
            debounce_count--;
        }
        else {
            pos_delta_ = pos - pos_last_;
            pos_last_ = pos;
            cout << pos_delta_.norm() << endl;
            if (pos_delta_.norm() > 0.004) {
                mat << R(0, 0), R(1, 0), R(2, 0), 0.,
                    R(0, 1), R(1, 1), R(2, 1), 0.,
                    R(0, 2), R(1, 2), R(2, 2), 0.,
                    pos.x(), pos.y(), pos.z(), 1.;
                key_frames_.push_back(mat);
            }
            debounce_count = 10;
        }
        initialized = false;
    }

    glPushMatrix();
    std::vector<GLdouble> Twc = {R(0, 0), R(1,0), R(2, 0), 0.,
                                R(0, 1), R(1, 1), R(2, 1), 0.,
                                R(0, 2), R(1, 2), R(2, 2), 0.,
                                pos.x(), pos.y(), pos.z(), 1.};
    glMultMatrixd(Twc.data());
    drawCam();
    glPopMatrix();
}

void SlamVisualizer::drawTraj(vector<Eigen::Vector3d> &traj){
    if(!traj_visible_)
        return;
    glLineWidth(2);
    glBegin(GL_LINES);
    glColor3f(0.f, 1.f, 0.f);
    for(size_t i=0; i<traj.size() - 1; i++){
        glVertex3d(traj[i].x(), traj[i].y(), traj[i].z());
        glVertex3d(traj[i+1].x(), traj[i+1].y(), traj[i+1].z());
    }
    glEnd();
}

void SlamVisualizer::drawCoordinate(){
    if(!coordinate_visible_)
        return;
    // 绘制坐标系
    glLineWidth(3);
    glBegin ( GL_LINES );
	glColor3f ( 1.0f,0.f,0.f );
	glVertex3f( 0,0,0 );
	glVertex3f( 1,0,0 );
	glColor3f( 0.f,1.0f,0.f);
	glVertex3f( 0,0,0 );
	glVertex3f( 0,1,0 );
	glColor3f( 0.f,0.f,1.f);
	glVertex3f( 0,0,0 );
	glVertex3f( 0,0,1 );
	glEnd();
}

void SlamVisualizer::drawKeyFrame(Eigen::Quaterniond& quat) {
    if (!keyframe_visible_)
        return;
    if (key_frames_.size() == 0)
        return;

    float scale_factor = 0.2;

    for (int i = 0; i < key_frames_.size() - 1; i++) {
        glLineWidth(0.2);
        glBegin(GL_LINES);
        glColor3f(1.0f, 0.f, 0.f);  // color
        glVertex3f(key_frames_[i](3, 0), key_frames_[i](3, 1), key_frames_[i](3, 2));  // 起点
        glVertex3f(key_frames_[i](3, 0) + scale_factor * key_frames_[i](0, 0), 
            key_frames_[i](3, 1) + scale_factor * key_frames_[i](1, 0),
            key_frames_[i](3, 2) + scale_factor * key_frames_[i](2, 0));  // 终点
        glColor3f(1.f, 1.0f, 0.f);
        glVertex3f(key_frames_[i](3, 0), key_frames_[i](3, 1), key_frames_[i](3, 2));
        glVertex3f(key_frames_[i](3, 0) + scale_factor * key_frames_[i](0, 1), 
            key_frames_[i](3, 1) + scale_factor * key_frames_[i](1, 1),
            key_frames_[i](3, 2) + scale_factor * key_frames_[i](2, 1));
        glColor3f(0.f, 0.f, 1.f);
        glVertex3f(key_frames_[i](3, 0), key_frames_[i](3, 1), key_frames_[i](3, 2));
        glVertex3f(key_frames_[i](3, 0) + scale_factor * key_frames_[i](0, 2), 
            key_frames_[i](3, 1) + scale_factor * key_frames_[i](1, 2), 
            key_frames_[i](3, 2) + scale_factor * key_frames_[i](2, 2));
    }
    glEnd();
}


void SlamVisualizer::displayImg(cv::Mat& originImg, cv::Mat& trackImg){
    if(!img_visible_)
        return;
    imageTexture_.Upload(originImg.data, GL_BGR, GL_UNSIGNED_BYTE);
        // 显示图像
    d_img_.Activate();
    glColor3f(1.0f, 1.0f, 1.0f); // 设置默认背景色，对于显示图片来说，不设置也没关系
    imageTexture_.RenderToViewportFlipY(); // 需要反转Y轴，否则输出是倒着的

    trackTexture_.Upload(trackImg.data, GL_BGR, GL_UNSIGNED_BYTE);
        // 显示图像
    d_track_.Activate();
    glColor3f(1.0f, 1.0f, 1.0f); // 设置默认背景色，对于显示图片来说，不设置也没关系
    trackTexture_.RenderToViewportFlipY(); // 需要反转Y轴，否则输出是倒着的
}

void SlamVisualizer::displayPcs(vector<Eigen::Vector3d>& pcs, Eigen::Vector3d& pos, Eigen::Quaterniond& quat, 
    vector<vector<Eigen::Vector3d>>& keyframe_pcs, vector <vector<Eigen::Vector3d>>& totalframe_pcs) {
    if (!pc_visible_)
        return;

    vector<Eigen::Vector3d> key_frame;

    Eigen::Matrix3d matrix = quat.toRotationMatrix();

    for (int i = 0; i < pcs.size() - 1; i++) {
        glColor3f(1, 1, 0);
        glPointSize(2);
        glBegin(GL_POINTS);
        // 将vec与matrix相乘，并将结果添加到结果向量中  
        pcs[i] = matrix * pcs[i];
        glVertex3d(pcs[i].x() + pos.x(), pcs[i].y() + pos.y(), 2 + pos.z());
        if (debounce_count == 0) {
            key_frame.push_back(Eigen::Vector3d(pcs[i].x() + pos.x(), pcs[i].y() + pos.y(), 2 + pos.z()));
        }
    }
    glEnd();

    if (keyframe_pcs.size() > 0) {
        for (int i = 0; i < (keyframe_pcs.size() - 1); i++) {
            for (int j = 0; j < (keyframe_pcs[i].size() - 1); j++) {
                glColor3f(0, 1, 0);
                glPointSize(2);
                glBegin(GL_POINTS);
                glVertex3d(keyframe_pcs[i][j].x(), keyframe_pcs[i][j].y(), keyframe_pcs[i][j].z());
            }
        }
        glEnd();
    }

    cout << keyframe_pcs.size() << endl;

    if (debounce_count == 0)
        keyframe_pcs.push_back(key_frame);
}

void SlamVisualizer::displayData(Eigen::Vector3d &pos, Eigen::Quaterniond& quat){
    VecXd tmp_pose, tmp_euler;
    tmp_pose.vec_ = pos;
    tmp_euler.vec_ = quat.matrix().eulerAngles(2, 1, 0);// YPR, quat是否需要转置？

    tmp_euler.vec_  *= (180 / M_PI);
    data_set_[0] = tmp_pose;
    data_set_[1] = tmp_euler;
}

void SlamVisualizer::registerUICallback(){
    camera_visible_ = ui_set_[0] ? true : false;
    traj_visible_ = ui_set_[1] ? true : false;
    img_visible_ = ui_set_[2] ? true : false;
    pc_visible_ = ui_set_[3] ? true : false;
    keyframe_visible_ = ui_set_[4] ? true : false;
    coordinate_visible_ = ui_set_[5] ? true : false;
    
    if(pangolin::Pushed(ui_set_[6]))
        d_cam_.SaveOnRender("map");
    
    if(pangolin::Pushed(ui_set_[7]))
        pangolin::SaveWindowOnRender("win");
}
