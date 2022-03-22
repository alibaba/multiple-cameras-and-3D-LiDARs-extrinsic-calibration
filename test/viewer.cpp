#include "viewer.h"


Viewer::Viewer(const std::vector<Eigen::Matrix4d> &v_cam_pose)
    : b_finish_request_(false),
      b_finished_(true),
      b_stopped_(true),
      b_stopp_request_(false),
      viewpoint_x_(0),
      viewpoint_y_(-0.7),
      viewpoint_z_(-1.8),
      viewpoint_f_(500),
      point_size_(2),
      keyframe_size_(0.05),
      keyframe_line_width_(4),
      graph_line_width_(1.0),
      camera_size_(1.0),
      camera_line_width_(5) {
  
  T_curr_pose_w_c_ = Eigen::Matrix4d::Zero();
  T_target_pose_w_c_ = Eigen::Matrix4d::Zero();
  v_T_poses_w_c_ = v_cam_pose;

}

Viewer::Viewer()
    : b_finish_request_(false),
      b_finished_(true),
      b_stopped_(true),
      b_stopp_request_(false) {
  
  viewpoint_x_ = 0;
  viewpoint_y_ = -0.7;
  viewpoint_z_ = -1.8;
  viewpoint_f_ = 500;

  point_size_ = 2;
  keyframe_size_ = 0.05;
  keyframe_line_width_ = 4;
  graph_line_width_ = 1.0;
  camera_size_ = 1.0;
  camera_line_width_ = 5;

  T_curr_pose_w_c_ = Eigen::Matrix4d::Zero();
  T_target_pose_w_c_ = Eigen::Matrix4d::Zero();
  v_T_poses_w_c_ = std::vector<Eigen::Matrix4d>(4, Eigen::Matrix4d::Zero());
}

void Viewer::run() {
  b_finished_ = false;
  b_stopped_ = false;

  pangolin::CreateWindowAndBind("Pose Graph: Pose Viewer", 1024, 768);

  // 3D Mouse handler requires depth testing to be enabled
  glEnable(GL_DEPTH_TEST);

  // Issue specific OpenGl we might need
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0,
                                          pangolin::Attach::Pix(175));
  pangolin::Var<bool> menu_follow_cam("menu.Follow Camera", false, true);
  pangolin::Var<bool> menu_show_kfmes("menu.Show KeyFrames", true, true);
  pangolin::Var<bool> menu_show_graph("menu.Show Graph", true, true);
  pangolin::Var<bool> menu_show_map("menu.Show Map point", true, true);
  pangolin::Var<bool> menu_reset("menu.Reset", false, false);

  // Define Camera Render Object (for view / scene browsing)
  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1024, 768, viewpoint_f_, viewpoint_f_, 512,
                                 389, 0.1, 1000),
      pangolin::ModelViewLookAt(viewpoint_x_, viewpoint_y_, viewpoint_z_, 0, 0,
                                0, 0.0, -1.0, 0.0));

  // Add named OpenGL viewport to window and provide 3D Handler
  pangolin::View &d_cam = pangolin::CreateDisplay()
                              .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175),
                                         1.0, -1024.0f / 768.0f)
                              .SetHandler(new pangolin::Handler3D(s_cam));

  pangolin::OpenGlMatrix Twc;
  Twc.SetIdentity();

  bool b_follow = true;

  while (1) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (menu_follow_cam && b_follow) {
      s_cam.Follow(Twc);
    } else if (menu_follow_cam && !b_follow) {
      s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(
          viewpoint_x_, viewpoint_y_, viewpoint_z_, 0, 0, 0, 0.0, -1.0, 0.0));
      s_cam.Follow(Twc);
      b_follow = true;
    } else if (!menu_follow_cam && b_follow) {
      b_follow = false;
    }

    // draw current camera pose
    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    // draw keyframes and pose graph
    if (menu_show_kfmes || menu_show_graph) {
      // get camera pose in pangolin format
      // getCurrentOpenGLCameraMatrix(Twc);
      // drawCurrentCamera(Twc);
      // drawTargetCamera(KeyFrameColor::RED_KEYFRAME, T_target_pose_w_c_.cast<float>());
      drawKeyFrames(true, true);
    }

    if (menu_show_map) {
      drawMapPoints();
      drawPnPPoints();
    }

    pangolin::FinishFrame();

    if (menu_reset) {
      menu_show_graph = true;
      menu_show_kfmes = true;
      b_follow = true;
      menu_follow_cam = true;
      menu_reset = false;
    }

    if (stop()) {
      while (isStopped()) {
        usleep(3000);
      }
    }

    if (checkFinish()) break;
  }

  setFinish();
}

void Viewer::requestFinish() {
  std::unique_lock<std::mutex> lock(mutex_finish_);
  b_finish_request_ = true;
}

bool Viewer::checkFinish() {
  std::unique_lock<std::mutex> lock(mutex_finish_);
  return b_finish_request_;
}

void Viewer::setFinish() {
  std::unique_lock<std::mutex> lock(mutex_finish_);
  b_finished_ = true;
}

bool Viewer::isFinished() {
  std::unique_lock<std::mutex> lock(mutex_finish_);
  return b_finished_;
}

void Viewer::requestStop() {
  std::unique_lock<std::mutex> lock(mutex_stop_);
  if (!b_stopped_) b_stopp_request_ = true;
}

bool Viewer::isStopped() {
  std::unique_lock<std::mutex> lock(mutex_stop_);
  return b_stopped_;
}

bool Viewer::stop() {
  std::unique_lock<std::mutex> lock(mutex_stop_);
  std::unique_lock<std::mutex> lock2(mutex_finish_);

  if (b_finish_request_)
    return false;
  else if (b_stopp_request_) {
    b_stopped_ = true;
    b_stopp_request_ = false;
    return true;
  }

  return false;
}

void Viewer::release() {
  std::unique_lock<std::mutex> lock(mutex_stop_);
  b_stopped_ = false;
}

void Viewer::drawKeyFrames(const bool bDrawKF, const bool bDrawGraph) {
  std::unique_lock<std::mutex> lock(mutex_camera_);

  const float &w = keyframe_size_;
  const float h = w * 0.75;
  const float z = w * 0.6;

  int kfrm_num = v_T_poses_w_c_.size();

  std::vector<KeyFrameColor> v_colors = {
      KeyFrameColor::BLACK_KEYFRAME, KeyFrameColor::BLUE_KEYFRAME,
      KeyFrameColor::RED_KEYFRAME, KeyFrameColor::GREEN_KEYFRAME};

  for (int i = 0; i < kfrm_num; i++) {
    // KeyFrameColor color = v_colors[i % 4];
    KeyFrameColor color = KeyFrameColor::BLUE_KEYFRAME;
    Eigen::Matrix4f T_w_c = v_T_poses_w_c_[i].cast<float>();

    glPushMatrix();
    glMultMatrixf((GLfloat *)(T_w_c.data()));

    glLineWidth(keyframe_line_width_);
    // blue color
    switch (color) {
      case KeyFrameColor::BLACK_KEYFRAME:
        // black color
        glColor3f(1.0f, 0.0f, 0.0f);
        break;
      case KeyFrameColor::RED_KEYFRAME:
        // red color
        glColor3f(1.0f, 0.0f, 0.0f);
        break;
      case KeyFrameColor::BLUE_KEYFRAME:
        // blue color
        glColor3f(0.0f, 0.0f, 1.0f);
        break;
      case KeyFrameColor::GREEN_KEYFRAME:
        // green color
        glColor3f(0.0f, 1.0f, 0.0f);
        break;
    }
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(w, h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(w, -h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(-w, -h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(-w, h, z);

    glVertex3f(w, h, z);
    glVertex3f(w, -h, z);

    glVertex3f(-w, h, z);
    glVertex3f(-w, -h, z);

    glVertex3f(-w, h, z);
    glVertex3f(w, h, z);

    glVertex3f(-w, -h, z);
    glVertex3f(w, -h, z);
    glEnd();

    glPopMatrix();
  }

  if (bDrawGraph) {
    glLineWidth(graph_line_width_);
    glColor4f(0.0f, 1.0f, 0.0f, 0.6f);
    glBegin(GL_LINES);

    for (int i = 0; i < kfrm_num - 1; i++) {
      Eigen::Vector3f src_cam_center =
          v_T_poses_w_c_[i].block<3, 1>(0, 3).cast<float>();
      Eigen::Vector3f dst_cam_center =
          v_T_poses_w_c_[i + 1].block<3, 1>(0, 3).cast<float>();

      // green graph edge for sequential edge
      glColor4f(0.0f, 1.0f, 0.0f, 0.6f);

      glBegin(GL_LINES);

      glVertex3f(src_cam_center[0], src_cam_center[1], src_cam_center[2]);
      glVertex3f(dst_cam_center[0], dst_cam_center[1], dst_cam_center[2]);

      glEnd();
    }
    glEnd();
  }
}

void Viewer::setCurrentMapPoints(
    const std::vector<Eigen::Vector3d> &v_map_points) {
  std::unique_lock<std::mutex> lock(mutex_map_points_);
  v_map_points_ = v_map_points;
}

void Viewer::setPnPPoints(const std::vector<Eigen::Vector3d> &v_map_points) {
  std::unique_lock<std::mutex> lock(mutex_map_points_);
  v_pnp_points_ = v_map_points;
}

void Viewer::drawMapPoints() {
  std::unique_lock<std::mutex> lock(mutex_map_points_);

  if (v_map_points_.empty()) {
    return;
  }

  glPointSize(point_size_);
  glBegin(GL_POINTS);
  glColor3f(0.0, 0.0, 0.0);
  for (size_t i = 0, iend = v_map_points_.size(); i < iend; ++i) {
    float x = v_map_points_[i][0];
    float y = v_map_points_[i][1];
    float z = v_map_points_[i][2];

    glVertex3f(x, y, z);
  }

  glEnd();
}

void Viewer::drawPnPPoints() {
  std::unique_lock<std::mutex> lock(mutex_map_points_);

  if (v_pnp_points_.empty()) {
    return;
  }

  glPointSize(6);
  glBegin(GL_POINTS);
  glColor3f(1.0, 0.0, 0.0);
  for (size_t i = 0, iend = v_pnp_points_.size(); i < iend; ++i) {
    float x = v_pnp_points_[i][0];
    float y = v_pnp_points_[i][1];
    float z = v_pnp_points_[i][2];

    glVertex3f(x, y, z);
  }

  glEnd();
}

void Viewer::drawCurrentCamera(pangolin::OpenGlMatrix &Twc) {
  const float &w = camera_size_;
  const float h = w * 0.75;
  const float z = w * 0.6;

  glPushMatrix();

#ifdef HAVE_GLES
  glMultMatrixf(Twc.m);
#else
  glMultMatrixd(Twc.m);
#endif

  glLineWidth(camera_line_width_);
  glColor3f(0.0f, 1.0f, 0.0f);
  glBegin(GL_LINES);
  glVertex3f(0, 0, 0);
  glVertex3f(w, h, z);
  glVertex3f(0, 0, 0);
  glVertex3f(w, -h, z);
  glVertex3f(0, 0, 0);
  glVertex3f(-w, -h, z);
  glVertex3f(0, 0, 0);
  glVertex3f(-w, h, z);

  glVertex3f(w, h, z);
  glVertex3f(w, -h, z);

  glVertex3f(-w, h, z);
  glVertex3f(-w, -h, z);

  glVertex3f(-w, h, z);
  glVertex3f(w, h, z);

  glVertex3f(-w, -h, z);
  glVertex3f(w, -h, z);
  glEnd();

  glPopMatrix();
}

void Viewer::drawTargetCamera(const KeyFrameColor &kyfrm_color,
                              const Eigen::Matrix4f &twc) {
  const float &w = camera_size_;
  const float h = w * 0.75;
  const float z = w * 0.6;

  glPushMatrix();

  glMultMatrixf((GLfloat *)(twc.data()));

  glLineWidth(camera_line_width_);

  switch (kyfrm_color) {
    case KeyFrameColor::BLACK_KEYFRAME:
      // black color
      glColor3f(0.0f, 0.0f, 0.0f);
      break;
    case KeyFrameColor::RED_KEYFRAME:
      // red color
      glColor3f(1.0f, 0.0f, 0.0f);
      break;
    case KeyFrameColor::BLUE_KEYFRAME:
      // blue color
      glColor3f(0.0f, 0.0f, 1.0f);
      break;
    case KeyFrameColor::GREEN_KEYFRAME:
      // green color
      glColor3f(0.0f, 1.0f, 0.0f);
      break;
  }

  glBegin(GL_LINES);
  glVertex3f(0, 0, 0);
  glVertex3f(w, h, z);
  glVertex3f(0, 0, 0);
  glVertex3f(w, -h, z);
  glVertex3f(0, 0, 0);
  glVertex3f(-w, -h, z);
  glVertex3f(0, 0, 0);
  glVertex3f(-w, h, z);

  glVertex3f(w, h, z);
  glVertex3f(w, -h, z);

  glVertex3f(-w, h, z);
  glVertex3f(-w, -h, z);

  glVertex3f(-w, h, z);
  glVertex3f(w, h, z);

  glVertex3f(-w, -h, z);
  glVertex3f(w, -h, z);
  glEnd();

  glPopMatrix();
}

void Viewer::setCurrentCameraPose(const Eigen::Matrix4d &Twc) {
  std::unique_lock<std::mutex> lock(mutex_camera_);
  T_curr_pose_w_c_ = Twc;
}

void Viewer::setTargetCameraPose(const Eigen::Matrix4d &Twc) {
  std::unique_lock<std::mutex> lock(mutex_camera_);
  T_target_pose_w_c_ = Twc;
}

void Viewer::getCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M) {
  if (!T_curr_pose_w_c_.hasNaN()) {
    Eigen::Matrix3d Rwc = Eigen::Matrix3d::Identity();
    Eigen::Vector3d twc = Eigen::Vector3d::Zero();
    {
      std::unique_lock<std::mutex> lock(mutex_camera_);
      Rwc = T_curr_pose_w_c_.block<3, 3>(0, 0);
      twc = T_curr_pose_w_c_.block<3, 1>(0, 3);
    }

    M.m[0] = Rwc(0, 0);
    M.m[1] = Rwc(1, 0);
    M.m[2] = Rwc(2, 0);
    M.m[3] = 0.0;

    M.m[4] = Rwc(0, 1);
    M.m[5] = Rwc(1, 1);
    M.m[6] = Rwc(2, 1);
    M.m[7] = 0.0;

    M.m[8] = Rwc(0, 2);
    M.m[9] = Rwc(1, 2);
    M.m[10] = Rwc(2, 2);
    M.m[11] = 0.0;

    M.m[12] = twc(0);
    M.m[13] = twc(1);
    M.m[14] = twc(2);
    M.m[15] = 1.0;
  } else {
    // ERROR_STREAM("Current camera pose is empty");
    M.SetIdentity();
  }
}

