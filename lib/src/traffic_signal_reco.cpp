#include "traffic_signal_reco.hpp"
namespace signal_reco {

/*** Function ***/
/*** コンストラクタ ***/
SignalReco::SignalReco()
{
  // initImgPcdName("../cfg/file_img_pcd_name.ini");
  // initImgPcdName("/home/revast/workspace/ryusei/traffic_signal_reco_lib/cfg/file_img_pcd_name.ini");
  // initParam("../cfg/parameter.ini");
  initParam("/home/revast/workspace/ryusei/traffic_signal_reco_lib/cfg/parameter.ini");
}
/*** デストラクタ ***/
SignalReco::~SignalReco()
{
}

void SignalReco::initImgPcdName(const string &path)
{
  std::ifstream ifs(path);
  if(!ifs.is_open()) {
    std::cerr << "Failed to open file_img_pcd_name.ini" << std::endl;
    return;
  }
  std::string line;
  while (std::getline(ifs, line)) {
    // 空行や';'で始まる行はスキップ
    if (line.empty() || line[0] == ';') continue;
    std::istringstream iss(line);
    // 文字列が格納されていることを確認
    if (!(iss >> img_path >> pcd_path >> cam_yaml_path)) {
      std::cerr << "Invalid line format: " << line << std::endl;
      continue;
    }
    cout << "img_path : " << img_path << endl;
    cout << "pcd_path : " << pcd_path << endl;
    cout << "cam_yaml_path : " << cam_yaml_path << endl;
  }
}

void SignalReco::initParam(const string &path)
{
  prop::ptree pt;
  prop::ini_parser::read_ini(path, pt);
  X_DIFF = pt.get<double>("X_DIFF");
  Y_DIFF = pt.get<double>("Y_DIFF");
  Z_DIFF = pt.get<double>("Z_DIFF");
  ROLL = pt.get<double>("ROLL");
  PITCH = pt.get<double>("PITCH");
  YAW = pt.get<double>("YAW");
  DETECT_HEIGHT_MIN = pt.get<double>("DETECT_HEIGHT_MIN");
  DETECT_HEIGHT_MAX = pt.get<double>("DETECT_HEIGHT_MAX");
  LIDAR_HEIGHT = pt.get<double>("LIDAR_HEIGHT");
  REFLECT_THRESH = pt.get<double>("REFLECT_THRESH");
  MIN_RANGE = pt.get<double>("MIN_RANGE");
  MAX_RANGE = pt.get<double>("MAX_RANGE");
  MAX_REFLECT = pt.get<double>("MAX_REFLECT");
  MIN_PIX_NUM_SIGN = pt.get<int>("MIN_PIX_NUM_SIGN");
  MAX_PIX_NUM_SIGN = pt.get<int>("MAX_PIX_NUM_SIGN");
  MIN_ASPECT_RATIO_SIGN = pt.get<double>("MIN_ASPECT_RATIO_SIGN");
  MAX_ASPECT_RATIO_SIGN = pt.get<double>("MAX_ASPECT_RATIO_SIGN");
  SIGNAL_RECO_REGION_WIDTH = pt.get<float>("SIGNAL_RECO_REGION_WIDTH");
  SIGNAL_RECO_REGION_HEIGHT = pt.get<float>("SIGNAL_RECO_REGION_HEIGHT");
  MIN_H_RED_01 = pt.get<int>("MIN_H_RED_01");
  MAX_H_RED_01 = pt.get<int>("MAX_H_RED_01");
  MIN_H_RED_02 = pt.get<int>("MIN_H_RED_02");
  MAX_H_RED_02 = pt.get<int>("MAX_H_RED_02");
  MIN_S_RED = pt.get<int>("MIN_S_RED");
  MAX_S_RED = pt.get<int>("MAX_S_RED");
  MIN_V_RED = pt.get<int>("MIN_V_RED");
  MAX_V_RED = pt.get<int>("MAX_V_RED");
  MIN_H_GREEN = pt.get<int>("MIN_H_GREEN");
  MAX_H_GREEN = pt.get<int>("MAX_H_GREEN");
  MIN_S_GREEN = pt.get<int>("MIN_S_GREEN");
  MAX_S_GREEN = pt.get<int>("MAX_S_GREEN");
  MIN_V_GREEN = pt.get<int>("MIN_V_GREEN");
  MAX_V_GREEN = pt.get<int>("MAX_V_GREEN");
  MIN_H_YELLOW = pt.get<int>("MIN_H_YELLOW");
  MAX_H_YELLOW = pt.get<int>("MAX_H_YELLOW");
  MIN_S_YELLOW = pt.get<int>("MIN_S_YELLOW");
  MAX_S_YELLOW = pt.get<int>("MAX_S_YELLOW");
  MIN_V_YELLOW = pt.get<int>("MIN_V_YELLOW");
  MAX_V_YELLOW = pt.get<int>("MAX_V_YELLOW");
  MIN_PIX_NUM = pt.get<int>("MIN_PIX_NUM");
  MAX_PIX_NUM = pt.get<int>("MAX_PIX_NUM");
  MIN_ASPECT_RATIO = pt.get<double>("MIN_ASPECT_RATIO");
  MAX_ASPECT_RATIO = pt.get<double>("MAX_ASPECT_RATIO");
  YELLOW_PIX_TH = pt.get<int>("YELLOW_PIX_TH");

}

/* ファイル名を取得 */
void SignalReco::getFiles(const fs::path &path, const string &extension, vector<fs::path> &files)
{
  for(const fs::directory_entry &p : fs::directory_iterator(path)){
    if(!fs::is_directory(p)){
      if(p.path().extension().string() == extension){
        files.push_back(p);
      }
    }
  }
  sort(files.begin(), files.end());
}

/* pcdファイルを読み込む関数 */
bool SignalReco::loadPCD(const string &path, vector<LidarData> &points)
{
  if (pr::loadPointsFromLog(path, points)) {
      return true;
  } else {
      cerr << "Failed to load points from file." << endl;
      return false;
  }
}

void SignalReco::euler2Quaternion(float roll, float pitch, float yaw, float &q_w, float &q_x, float &q_y, float &q_z)
{
  double cos_roll = cos(roll / 2.0);
  double sin_roll = sin(roll / 2.0);
  double cos_pitch = cos(pitch / 2.0);
  double sin_pitch = sin(pitch / 2.0);
  double cos_yaw = cos(yaw / 2.0);
  double sin_yaw = sin(yaw / 2.0);
  q_w = cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw;
  q_x = sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw;
  q_y = cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw;
  q_z = cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw;
}

void SignalReco::rotatePoints(const vector<LidarData> &src, float roll, float pitch, float yaw, vector<LidarData> &dst)
{
  int sz = src.size();
  float qw, qx, qy, qz;
  euler2Quaternion(roll, pitch, yaw, qw, qx, qy, qz);
  qz = -qz;
  if(dst.size() != src.size()) dst.resize(sz);
  vector<float> r{
    (qw * qw) + (qx * qx) - (qy * qy) - (qz * qz), 2 * (qw * qz + qx * qy), 2 * (qx * qz - qw * qy),
    2 * (qx * qy - qw * qz), qw * qw - qx * qx + qy * qy - qz * qz, 2 * (qy * qz + qw * qx),
    2 * (qw * qy + qx * qz), 2 * (-qw * qx + qy * qz), qw * qw - qx * qx - qy * qy + qz * qz
  };
  for(int i = 0; i < sz; i++){
    float x = src[i].x, y = src[i].y, z = src[i].z;
    dst[i].x = r[0] * x + r[1] * y + r[2] * z;
    dst[i].y = r[3] * x + r[4] * y + r[5] * z;
    dst[i].z = r[6] * x + r[7] * y + r[8] * z;

    dst[i].x = dst[i].x - X_DIFF;
    dst[i].y = dst[i].y - Y_DIFF;
    dst[i].z = dst[i].z - Z_DIFF;

    dst[i].range = src[i].range;
    dst[i].reflectivity = src[i].reflectivity;
  }
}

void SignalReco::projectToImage(const vector<LidarData> &points, Mat &lidar_img, bool is_for_reco)
{
  /*** 視野角を解像度で割ると幅と高さのピクセル数が求まる ***/
  int width = cvRound(LIDAR_VIEW_ANGLE_H / LIDAR_RESOLUTION_H);
  int height = cvRound(LIDAR_VIEW_ANGLE_V / LIDAR_RESOLUTION_V);
  lidar_img = Mat(height, width, CV_32FC2, Scalar(.0, .0));
  
  /*** points[i]の水平角度と垂直角度を求める ***/
  int sz = points.size();
  for(int i = 0; i < sz; i++)
  {
    if(points[i].z < DETECT_HEIGHT_MIN) continue;
    double angle_h = atan2(points[i].y, points[i].x); 
    double angle_v = atan2(points[i].z - LIDAR_HEIGHT, sqrt(points[i].x * points[i].x + points[i].y * points[i].y));
    /*** 求めた角度が画像のどこの画素に対応するか求める ***/
    int x = cvRound(width / 2 - angle_h / LIDAR_RESOLUTION_H);
    int y = cvRound(height / 2 - angle_v / LIDAR_RESOLUTION_V);

    if(x < 0 || y < 0 || x >= width || y >= height) continue;
    lidar_img.at<Vec2f>(y, x) = Vec2f(points[i].reflectivity, points[i].range);
  }
  if(!is_for_reco) {
    height = cvRound(Deg2Rad(height) / LIDAR_RESOLUTION_H);
    resize(lidar_img, lidar_img, Size(width, height), 0, 0, INTER_NEAREST);
  }
}

void SignalReco::drawObjectsReflect(const Mat &lidar_data, Mat &img)
{
  auto remap = [](float val, float from_low, float from_high, float to_low, float to_high)
  {
    return (val - from_low) * (to_high - to_low) / (from_high - from_low) + to_low;
  };
  auto &lidar_img = lidar_data;
  if(img.cols != lidar_img.cols || img.rows != lidar_img.rows) img = Mat(lidar_img.size(),CV_8UC3, Scalar(.0, .0, .0));
  else img = Scalar(.0, .0, .0);
  for(int y = 0; y < img.rows; y++){
    for(int x = 0; x < img.cols; x++){
      float reflect = lidar_img.at<Vec2f>(y, x)[0];
      float range = lidar_img.at<Vec2f>(y, x)[1];
      if(reflect > MAX_REFLECT || reflect <= .0f) continue; // 必須
      if(range > MAX_RANGE || range <= MIN_RANGE) continue; // 必須
      if(reflect < REFLECT_THRESH) continue; // 閾値処理
      int val = (int)remap(reflect, 0.0f, (float)MAX_REFLECT, 30.0f, 255.0f);
      img.at<Vec3b>(y, x) = Vec3b(val, val, val);
    }
  }
}

void SignalReco::drawObjectsRange(const Mat &lidar_data, Mat &img)
{
  auto remap = [](float val, float from_low, float from_high, float to_low, float to_high)
  {
    return (val - from_low) * (to_high - to_low) / (from_high - from_low) + to_low;
  };
  auto &lidar_img = lidar_data;
  if(img.cols != lidar_img.cols || img.rows != lidar_img.rows) img = Mat(lidar_img.size(), CV_8UC3, Scalar(.0, .0, .0));
  else img = Scalar(.0, .0, .0);
  for(int y = 0; y < img.rows; y++){
    for(int x = 0; x < img.cols; x++){
      float range = lidar_img.at<Vec2f>(y, x)[1];
      if(range > MAX_RANGE || range <= MIN_RANGE) continue;
      int val = (int)remap(range, (float)MAX_RANGE, 0.0f, 30.0f, 255.0f);
      img.at<Vec3b>(y, x) = Vec3b(val, val, val);
    }
  }
}

void SignalReco::rectangleReflect(const Mat &lidar_img_ref, Mat &lidar_img_ref_bin, vector<vector<cv::Point2i>> &sign_rects_refimg_screen)
{
  sign_rects_refimg_screen.clear();
  cv::Mat gray;
  if (lidar_img_ref.channels() == 3) {
      cv::cvtColor(lidar_img_ref, gray, cv::COLOR_BGR2GRAY);
  } else {
      gray = lidar_img_ref;
  }
  // 二値化
  cv::threshold(gray, lidar_img_ref_bin, 0, 255, cv::THRESH_BINARY);
  // ラベリング処理を適用
  cv::Mat labels, stats, centroids;
  int num_labels = cv::connectedComponentsWithStats(lidar_img_ref_bin, labels, stats, centroids, 8, CV_32S);
  // 矩形情報をsign_rectsに格納
  for (int i = 1; i < num_labels; i++) {
    int x = stats.at<int>(i, cv::CC_STAT_LEFT);
    int y = stats.at<int>(i, cv::CC_STAT_TOP);
    int w = stats.at<int>(i, cv::CC_STAT_WIDTH);
    int h = stats.at<int>(i, cv::CC_STAT_HEIGHT);
    int area = stats.at<int>(i, cv::CC_STAT_AREA);
    if (area < MIN_PIX_NUM_SIGN || area > MAX_PIX_NUM_SIGN) continue;
    double aspect_ratio = static_cast<double>(w) / static_cast<double>(h);
    if (aspect_ratio < MIN_ASPECT_RATIO_SIGN || aspect_ratio > MAX_ASPECT_RATIO_SIGN) continue;
    // 矩形の4点を計算
    std::vector<cv::Point2i> rect_points;
    rect_points.push_back(cv::Point2i(x, y));             // 左上
    rect_points.push_back(cv::Point2i(x, y + h));         // 左下
    rect_points.push_back(cv::Point2i(x + w, y + h));     // 右下
    rect_points.push_back(cv::Point2i(x + w, y));         // 右上
    // sign_rectsに矩形情報を格納
    sign_rects_refimg_screen.push_back(rect_points);
  }
  // 矩形を描画
  for (int i = 0; i < sign_rects_refimg_screen.size(); i++) {
    cv::rectangle(lidar_img_ref, sign_rects_refimg_screen[i][0], sign_rects_refimg_screen[i][2], cv::Scalar(0, 255, 255), 1);
  }
}

void SignalReco::screen2CenteredCoords(cv::Size image_size, const vector<vector<cv::Point2i>> &sign_rect_refimg_screen, vector<vector<cv::Point2i>> &sign_rects_refimg_centered_screen)
{
  sign_rects_refimg_centered_screen.clear();
  int width = image_size.width;
  int height = image_size.height;
  for (int i = 0; i < sign_rect_refimg_screen.size(); i++) {
    vector<cv::Point2i> centered_rect;
    centered_rect.resize(4);
    centered_rect[0].x = height / 2 - sign_rect_refimg_screen[i][0].y;
    centered_rect[0].y = -(sign_rect_refimg_screen[i][0].x - width / 2);
    centered_rect[1].x = height / 2 - sign_rect_refimg_screen[i][1].y;
    centered_rect[1].y = -(sign_rect_refimg_screen[i][1].x - width / 2);
    centered_rect[2].x = height / 2 - sign_rect_refimg_screen[i][2].y;
    centered_rect[2].y = -(sign_rect_refimg_screen[i][2].x - width / 2);
    centered_rect[3].x = height / 2 - sign_rect_refimg_screen[i][3].y;
    centered_rect[3].y = -(sign_rect_refimg_screen[i][3].x - width / 2);
    sign_rects_refimg_centered_screen.push_back(centered_rect);
  }
}

void SignalReco::centeredScreen2RobotCoords(const Mat &lidar_img, const vector<vector<cv::Point2i>> &rects, const vector<vector<cv::Point2i>> &s, vector<vector<cv::Point3f>> &r)
{
  r.clear();
  // 入力画像が空でないか確認
  if (lidar_img.empty()) {
      std::cerr << "Error: lidar_img is empty." << std::endl;
      return;
  }
  // lidar_imgの型を確認
  if (lidar_img.type() != CV_32FC2) {
      std::cerr << "Error: lidar_img must be of type CV_32FC2." << std::endl;
      return;
  }

  r.resize(s.size()); // 外側のベクトルのサイズを確保
  for(int i = 0; i < s.size(); i++) {
    r[i].resize(s[i].size()); // 内側のベクトルのサイズを確保
    for (int j = 0; j < s[i].size(); j++) {  // 範囲0〜3
      // 範囲チェック
      if (i >= rects.size() || j >= rects[i].size()) {
        std::cerr << "Index out of bounds: i=" << i << ", j=" << j << std::endl;
        continue;
      }
      if (rects[i][j].x < 0 || rects[i][j].x >= lidar_img.cols ||
          rects[i][j].y < 0 || rects[i][j].y >= lidar_img.rows) {
            std::cerr << "Error: Index out of bounds: x=" << rects[i][j].x
                      << ", y=" << rects[i][j].y << std::endl;
            continue;
      }
      float range;
      if(j == 0)       range = lidar_img.at<Vec2f>(rects[i][j].y + 1, rects[i][j].x + 1)[1];
      else if (j == 1) range = lidar_img.at<Vec2f>(rects[i][j].y - 1, rects[i][j].x + 1)[1];
      else if (j == 2) range = lidar_img.at<Vec2f>(rects[i][j].y - 1, rects[i][j].x - 1)[1];
      else if (j == 3) range = lidar_img.at<Vec2f>(rects[i][j].y + 1, rects[i][j].x - 1)[1];
      else cerr << "Error: j is out of range" << endl;

      float angle_h = s[i][j].y * LIDAR_RESOLUTION_H;
      float angle_v = s[i][j].x * LIDAR_RESOLUTION_V;
      r[i][j].x = range * cos(angle_v) * cos(angle_h);
      r[i][j].y = range * cos(angle_v) * sin(angle_h);
      r[i][j].z = range * sin(angle_v);
    }
  }
}

void SignalReco::perspectiveProjectionModel(const vector<vector<cv::Point3f>> &sign_rect_points_robot, vector<vector<cv::Point2i>> &sign_rects_perspective)
{
  sign_rects_perspective.clear();
  double fx = camera_params.at<double>(0, 0);
  double fy = camera_params.at<double>(1, 1);
  for (int i = 0; i < sign_rect_points_robot.size(); i++) {
    vector<cv::Point2i> rect_points;
    for (int j = 0; j < sign_rect_points_robot[i].size(); j++) {
      cv::Point2i point;
      point.x = fx * sign_rect_points_robot[i][j].z / sign_rect_points_robot[i][j].x;
      point.y = fy * sign_rect_points_robot[i][j].y / sign_rect_points_robot[i][j].x;
      rect_points.push_back(point);
    }
    sign_rects_perspective.push_back(rect_points);
  }
}

void SignalReco::centeredScreen2ScreenCoords(cv::Size image_size, vector<vector<cv::Point2i>> &rects_r_sc, vector<vector<cv::Point2i>> &rects_sc)
{
  rects_sc.clear();
  rects_sc.resize(rects_r_sc.size());
  for (int i = 0; i < rects_sc.size(); i++) {
    vector<cv::Point2i> rect_points; 
    rects_sc[i].resize(rects_r_sc[i].size());
    for (int j = 0; j < sign_rects_camimg_perspective[i].size(); j++) {
      cv::Point2i point;
      point.x = -rects_r_sc[i][j].y + image_size.width / 2;
      point.y = -rects_r_sc[i][j].x + image_size.height / 2;
      rect_points.push_back(point);
    }
    rects_sc[i] = rect_points;
  }
}

void SignalReco::drawSignOnCameraImg(const Mat &camera_img, const vector<vector<cv::Point2i>> &sign_rects)
{
  // 矩形を描画
  for (int i = 0; i < sign_rects.size(); i++) {
    cv::rectangle(camera_img, sign_rects[i][0], sign_rects[i][2], cv::Scalar(255, 0, 210), 2);
  }
}

void SignalReco::storeSignalRects(const vector<vector<cv::Point2i>> &sign_rects, vector<vector<Point2i>> &signal_rects)
{
  signal_rects.clear();
  signal_rects.resize(sign_rects.size());
  for (int i = 0; i < sign_rects.size(); i++) {
    vector<cv::Point2i> rects; 
    signal_rects[i].resize(sign_rects[i].size());
    for (int j = 0; j < sign_rects[i].size(); j++) {
      cv::Point2i point;
      int sign_width = sign_rects[i][2].x - sign_rects[i][0].x;
      int sign_height = sign_rects[i][2].y - sign_rects[i][0].y;
      int signal_width = sign_width * SIGNAL_RECO_REGION_WIDTH;
      int signal_height = sign_height * SIGNAL_RECO_REGION_HEIGHT;
      if (j == 0) {
        point.x = sign_rects[i][2].x + sign_width / 2 - signal_width;
        point.y = sign_rects[i][0].y + sign_height / 2 - signal_height / 2;
      } else if (j == 1) {
        point.x = sign_rects[i][2].x + sign_width / 2 - signal_width;
        point.y = sign_rects[i][0].y + sign_height / 2 + signal_height / 2;
      } else if (j == 2) {
        point.x = sign_rects[i][0].x + sign_width / 2;
        point.y = sign_rects[i][0].y + sign_height / 2 + signal_height / 2;
      } else if (j == 3) {
        point.x = sign_rects[i][0].y + sign_height + signal_height;
        point.y = sign_rects[i][0].y + sign_height / 2 - signal_height / 2;
      }
      rects.push_back(point); 
    }
    signal_rects[i] = rects;
  }
}

void SignalReco::drawSignalRectsOnCameraImg(Mat &camera_img, const vector<vector<Point2i>> &signal_rects)
{
  for (int i = 0; i < signal_rects.size(); i++) {
    cv::rectangle(camera_img, signal_rects[i][0], signal_rects[i][2], cv::Scalar(50, 255, 0), 2);
  }
}

void SignalReco::cropSignalRectsFromCameraImg(const Mat &camera_img, const vector<vector<Point2i>> &signal_rects, vector<Mat> &signal_imgs)
{
  signal_imgs.clear();
  for (int i = 0; i < signal_rects.size(); i++) {
    cv::Rect roi(signal_rects[i][0], signal_rects[i][2]);
    if (roi.x < 0 || roi.y < 0 || roi.x + roi.width > camera_img.cols || roi.y + roi.height > camera_img.rows) {
      std::cerr << "Error: ROI is out of bounds." << std::endl;
      continue;
    }
    Mat cropped_img = camera_img(roi);
    signal_imgs.push_back(cropped_img);
  }
}

void SignalReco::rgb2HSV(const vector<Mat> &src, vector<Mat> &dst)
{
  dst.clear();
  for (const auto &img : src) {
    Mat img_hsv;
    cvtColor(img, img_hsv, COLOR_BGR2HSV);
    dst.push_back(img_hsv);
  }
}

void SignalReco::extractColor(const vector<Mat> &src, vector<Mat> &dst, bool is_red)
{
  dst.clear();
  for (const auto &img : src) {
    Mat img_extract;
    if (is_red) {
      inRange(img, Scalar(MIN_H_RED_01, MIN_S_RED, MIN_V_RED), Scalar(MAX_H_RED_01, MAX_S_RED, MAX_V_RED), img_extract);
      Mat img_extract2;
      inRange(img, Scalar(MIN_H_RED_02, MIN_S_RED, MIN_V_RED), Scalar(MAX_H_RED_02, MAX_S_RED, MAX_V_RED), img_extract2);
      img_extract |= img_extract2;
    } else {
      inRange(img, Scalar(MIN_H_GREEN, MIN_S_GREEN, MIN_V_GREEN), Scalar(MAX_H_GREEN, MAX_S_GREEN, MAX_V_GREEN), img_extract);
    }
    dst.push_back(img_extract);
  }
}

void SignalReco::medianImgs(const vector<Mat> &src, vector<Mat> &dst)
{
  dst.clear();
  for (const auto &img : src) {
    Mat img_median;
    medianBlur(img, img_median, 3);
    dst.push_back(img_median);
  }
}

void SignalReco::dilateImgs(const vector<Mat> &src, vector<Mat> &dst)
{
  dst.clear();
  for (const auto &img : src) {
    Mat img_dilated;
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    dilate(img, img_dilated, kernel);
    dst.push_back(img_dilated);
  }
}

void SignalReco::labeling(const vector<Mat> &src, vector<Mat> &dst, vector<vector<Mat>> &stats)
{
  dst.clear();
  stats.clear();
  for (const auto &img : src) {
    Mat img_label, img_stats, img_centroids;
    int num_labels = connectedComponentsWithStats(img, img_label, img_stats, img_centroids, 8, CV_32S);
    dst.push_back(img_label);

    vector<Mat> label_stats;
    for (int i = 1; i < num_labels; i++) {
      // ラベルの面積が最小値と最大値の範囲内であるか確認
      if (img_stats.at<int>(i, cv::CC_STAT_AREA) < MIN_PIX_NUM || img_stats.at<int>(i, cv::CC_STAT_AREA) > MAX_PIX_NUM) {
        continue;
      }
      // アスペクト比の計算
      double aspect_ratio = static_cast<double>(img_stats.at<int>(i, cv::CC_STAT_WIDTH)) / img_stats.at<int>(i, cv::CC_STAT_HEIGHT);
      if (aspect_ratio < MIN_ASPECT_RATIO || aspect_ratio > MAX_ASPECT_RATIO) {
        continue;
      }
      Mat stat = img_stats.row(i);
      label_stats.push_back(stat);
    }
    stats.push_back(label_stats); 
  }
}

void SignalReco::drawSignalCandidates(const vector<Mat> &src, const vector<vector<Mat>> &stats, bool is_red)
{
  for (int i = 0; i < stats.size(); i++) {
    for (int j = 0; j < stats[i].size(); j++) {
      int x = stats[i][j].at<int>(cv::CC_STAT_LEFT);
      int y = stats[i][j].at<int>(cv::CC_STAT_TOP);
      int w = stats[i][j].at<int>(cv::CC_STAT_WIDTH);
      int h = stats[i][j].at<int>(cv::CC_STAT_HEIGHT);
      if (is_red) cv::rectangle(src[i], cv::Point(x, y), cv::Point(x + w, y + h), Scalar(166, 103, 255), 2);
      else cv::rectangle(src[i], cv::Point(x, y), cv::Point(x + w, y + h), Scalar(247, 235, 74), 2);
    }
  }
}

void SignalReco::extractYellow(vector<Mat> &imgs, vector<vector<Mat>> &stats, vector<vector<vector<Mat>>> &imgs_ex_yellow)
{
  imgs_ex_yellow.clear();
  for (int i = 0; i < imgs.size(); i++) {
    vector<vector<Mat>> signal_imgs_yellow;
    for (int j = 0; j < stats.size(); j++) {
      vector<Mat> yellow_imgs;
      for (int k = 0; k < stats[j].size(); k++) {
        int x = stats[j][k].at<int>(cv::CC_STAT_LEFT);
        int y = stats[j][k].at<int>(cv::CC_STAT_TOP);
        int w = stats[j][k].at<int>(cv::CC_STAT_WIDTH);
        int h = stats[j][k].at<int>(cv::CC_STAT_HEIGHT);
        if (w <= 0 || h <= 0) continue; // 幅と高さが0以下の場合はスキップ
        Mat img_yellow = imgs[i](Rect(x, y, w, h));
        Mat img_yellow_bin = Mat::zeros(img_yellow.size(), CV_8UC1);
        imshow("img_yellow", img_yellow);
        Mat img_yellow_hsv;
        cvtColor(img_yellow, img_yellow_hsv, COLOR_BGR2HSV);
        inRange(img_yellow_hsv, Scalar(MIN_H_YELLOW, MIN_S_YELLOW, MIN_V_YELLOW), Scalar(MAX_H_YELLOW, MAX_S_YELLOW, MAX_V_YELLOW), img_yellow_bin);
        yellow_imgs.push_back(img_yellow_bin);
        // /*** 黄色の色付き画像を表示したいのであればコメントアウトを外す *********************************
        Mat img_yellow_color;
        bitwise_and(img_yellow, img_yellow, img_yellow_color, img_yellow_bin);
        imshow("img_yellow_color", img_yellow_color);
        // ***/
      }
      signal_imgs_yellow.push_back(yellow_imgs);
    }
    imgs_ex_yellow.push_back(signal_imgs_yellow);
  }
}

void SignalReco::labelingYellow(const vector<vector<vector<Mat>>> &imgs_ex_yellow, vector<vector<vector<Mat>>> &imgs_yellow_labeling, vector<vector<vector<Mat>>> &imgs_yellow_stats, int &num_figures, string &signal_state, bool is_red)
{
  // ラベリングされた二値化画像に番号を描画する処理を加える
  imgs_yellow_labeling.clear();
  imgs_yellow_stats.clear();
  for (const auto &signal_imgs_yellow : imgs_ex_yellow) {
    vector<vector<Mat>> signal_labeling;
    vector<vector<Mat>> signal_stats;
    for (const auto &yellow_imgs : signal_imgs_yellow) {
      vector<Mat> yellow_labeling;
      vector<Mat> yellow_stats;
      for (const auto &img_yellow : yellow_imgs) {
        Mat img_label, img_stats, img_centroids;
        int num_labels = connectedComponentsWithStats(img_yellow, img_label, img_stats, img_centroids, 8, CV_32S);
        Mat valid_stats;
        for (int l = 1; l < num_labels; ++l) {  
          int area = img_stats.at<int>(l, cv::CC_STAT_AREA);
          if (area >= YELLOW_PIX_TH) {
            // valid_stats.push_back(img_stats.row(l));
            // // ラベルの中心座標を取得
            // double x = img_centroids.at<double>(l, 0);
            // double y = img_centroids.at<double>(l, 1);

            // std::string label_text = std::to_string(l);
            // putText(img_yellow, label_text, cv::Point(static_cast<int>(x), static_cast<int>(y)), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
            num_figures++;
          }
        }
        if (valid_stats.rows > 0) {
          yellow_labeling.push_back(img_label);
          yellow_stats.push_back(valid_stats.clone());
        }
      }
      signal_labeling.push_back(yellow_labeling);
      signal_stats.push_back(yellow_stats);
    }
    imgs_yellow_labeling.push_back(signal_labeling);
    imgs_yellow_stats.push_back(signal_stats);
  }
  
  if (num_figures >= 1 && is_red) {
    signal_state = "RedLight";
  } else if (num_figures >= 1 && !is_red) {
    signal_state = "GreenLight";
  }
}

void SignalReco::drawRects(const vector<vector<Mat>> &imgs_stats, int num_figures, vector<Mat> &signal_imgs, bool is_red)
{
  for (int i = 0; i < imgs_stats.size(); i++) {
    for (int j = 0; j < imgs_stats[i].size(); j++) {
      if (num_figures > 0) {
        int x = imgs_stats[i][j].at<int>(cv::CC_STAT_LEFT);
        int y = imgs_stats[i][j].at<int>(cv::CC_STAT_TOP);
        int w = imgs_stats[i][j].at<int>(cv::CC_STAT_WIDTH);
        int h = imgs_stats[i][j].at<int>(cv::CC_STAT_HEIGHT);
        if (is_red) {
          rectangle(signal_imgs[i], Point(x, y), Point(x + w, y + h), Scalar(0, 0, 255), 2);
        } else {
          rectangle(signal_imgs[i], Point(x, y), Point(x + w, y + h), Scalar(0, 255, 0), 2);
        }
      }
    }
  }
}

void SignalReco::drawResult(Mat &camera_img, const string &signal_state)
{
  // 信号の状態を正方形でカメラ画像に描画
  int square_size = 50; // 正方形のサイズ
  if (signal_state == "RedLight") {
    rectangle(camera_img, Point(10, 10), Point(10 + square_size, 10 + square_size), Scalar(0, 0, 255), -1); // 赤色
    putText(camera_img, "Red Light", Point(10 + square_size + 10, 10 + square_size / 2 + 12), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 0, 255), 2);
  } else if (signal_state == "GreenLight") {
    rectangle(camera_img, Point(10, 10), Point(10 + square_size, 10 + square_size), Scalar(0, 255, 0), -1); // 緑色
    putText(camera_img, "Green Light", Point(10 + square_size + 10, 10 + square_size / 2 + 12), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 255, 0), 2);
  }
}

void SignalReco::loop_main()
{
  /*** 画像が読み込まれているか確認 ***/
  if (src_camera_img.empty()) {
    std::cerr << "Error: Could not read camera_img" << std::endl;
    return;
  }
  /*** 点群が読み込まれているか確認 **/
  if (src_points.empty()) {
    std::cerr << "Error: Could not read point cloud data" << std::endl;
    return;
  }
  /*** カメラキャリブレーションパラメータを取得 ***/
  cam_yaml_path = "/home/revast/workspace/ryusei/traffic_signal_reco_lib/parameter/streamcam.yaml";
  FileStorage fs(cam_yaml_path, FileStorage::READ);
  if(!fs.isOpened()) {
    cerr << "Failed to open camera YAML file: " << cam_yaml_path << endl;
  }
  Size sz((int)fs["image_width"], (int)fs["image_height"]);
  auto model = (string)fs["distortion_model"];
  fs["camera_matrix"] >> camera_params;
  fs["distortion_coefficients"] >> distortion_params;
  fs["rectification_matrix"] >> rectify_params;
  fs["projection_matrix"] >> projection_params;
  initUndistortRectifyMap(camera_params, distortion_params, rectify_params, projection_params, sz, CV_32FC1, map_x, map_y);
  /*** 変換処理、カメラ画像の歪み補正 ***/
  remap(src_camera_img, camera_img, map_x, map_y, INTER_LINEAR);
  rotatePoints(src_points, Deg2Rad(ROLL), Deg2Rad(PITCH), Deg2Rad(YAW), points);
  /*** 歩行者用信号機の横に付随する交通標識の検出 **********************************************************/
  projectToImage(points, lidar_img, true); // 反射強度画像、距離画像の作成
  // projectToImage(points, lidar_img_fov, false); // 反射強度画像、距離画像の作成（画像確認用）
  drawObjectsReflect(lidar_img, lidar_img_ref); // 反射強度画像の描画
  // drawObjectsReflect(lidar_img_fov, lidar_img_ref_fov); // 反射強度画像の描画（画像確認用）
  drawObjectsRange(lidar_img, lidar_img_range); // 距離画像の描画
  // drawObjectsRange(lidar_img_fov, lidar_img_range_fov); // 距離画像の描画（画像確認用）
  rectangleReflect(lidar_img_ref, lidar_img_ref_bin, sign_rects_refimg_screen);  // 反射強度画像の矩形領域を検出
  screen2CenteredCoords(lidar_img.size(), sign_rects_refimg_screen, sign_rects_refimg_centered_screen); // 矩形領域の座標系を正規スクリーン座標系に変換
  centeredScreen2RobotCoords(lidar_img, sign_rects_refimg_screen, sign_rects_refimg_centered_screen, sign_rect_points_robot); // 正規スクリーン座標系をカメラ座標系に変換
  perspectiveProjectionModel(sign_rect_points_robot, sign_rects_camimg_perspective); // 透視投影モデルを適用
  centeredScreen2ScreenCoords(camera_img.size(), sign_rects_camimg_perspective, sign_rects); // 正規スクリーン座標系をスクリーン座標系に変換
  // drawSignOnCameraImg(camera_img, sign_rects); // カメラ画像に標識の矩形を描画
  storeSignalRects(sign_rects, signal_rects); // 信号の矩形を格納
  // drawSignalRectsOnCameraImg(camera_img, signal_rects); // カメラ画像に信号の矩形を描画
  /*** 画像処理による歩行者用信号の色認識 **********************************************************************/
  cropSignalRectsFromCameraImg(camera_img, signal_rects, signal_imgs); // signal_rectsの座標情報を基にcamera_imgから信号の画像を切り出す
  rgb2HSV(signal_imgs, signal_imgs_hsv); // signal_imgsの画像をHSV色空間に変換
  extractColor(signal_imgs_hsv, imgs_extract_red, true); // 閾値から赤色を抽出
  extractColor(signal_imgs_hsv, imgs_extract_green, false); // 閾値から緑色を抽出
  medianImgs(imgs_extract_red, imgs_red_median); // メディアンフィルターによりごま塩ノイズを除去
  medianImgs(imgs_extract_green, imgs_green_median);
  dilateImgs(imgs_red_median, imgs_red_dilated); // 膨張処理
  dilateImgs(imgs_green_median, imgs_green_dilated);
  // imgs_extract_dilatedの画像にラベリング処理を適用する関数
  labeling(imgs_red_dilated, imgs_red_labeling, imgs_red_stats); // 赤色の画像にラベリング処理を適用
  labeling(imgs_green_dilated, imgs_green_labeling, imgs_green_stats); // 緑色の画像にラベリング処理を適用
  // drawSignalCandidates(signal_imgs, imgs_red_stats, true); // 候補領域に桃色の矩形を描画
  // drawSignalCandidates(signal_imgs, imgs_green_stats, false); // 候補領域に水色の矩形を描画
  imgs_red_ex_yellow.clear(); // 黄色の人型を格納するための変数を初期化
  imgs_green_ex_yellow.clear(); // 黄色の人型を格納するための変数を初期化
  imgs_red_ex_yellow_labeling.clear(); // 黄色の人型にラベリング処理を適用するための変数を初期化
  imgs_green_ex_yellow_labeling.clear(); // 黄色の人型にラベリング処理を適用するための変数を初期化
  imgs_red_ex_yellow_stats.clear(); // 黄色の人型の統計情報を格納するための変数を初期化
  imgs_green_ex_yellow_stats.clear(); // 黄色の人型の統計情報を格納するための変数を初期化
  extractYellow(signal_imgs, imgs_red_stats, imgs_red_ex_yellow); // 候補領域の中から黄色の人形を抽出
  extractYellow(signal_imgs, imgs_green_stats, imgs_green_ex_yellow); // 候補領域の中から黄色の人形を抽出
  num_figures_red = 0, num_figures_green = 0;
  signal_state = "";
  labelingYellow(imgs_red_ex_yellow, imgs_red_ex_yellow_labeling, imgs_red_ex_yellow_stats, num_figures_red, signal_state, true); // 黄色の人型にラベリング処理を適用　赤信号を判定
  labelingYellow(imgs_green_ex_yellow, imgs_green_ex_yellow_labeling, imgs_green_ex_yellow_stats, num_figures_green, signal_state, false); // 黄色の人型にラベリング処理を適用　青信号を判定
  drawRects(imgs_red_stats, num_figures_red, signal_imgs, true);
  drawRects(imgs_green_stats, num_figures_green, signal_imgs, false);
  drawResult(camera_img, signal_state); // 信号の状態をカメラ画像に描画
} // loop_main()

} // namespace signal_reco
