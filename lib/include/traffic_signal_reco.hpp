#ifndef TRAFFIC_SIGNAL_RECO_HPP_
#define TRAFFIC_SIGNAL_RECO_HPP_

/*** Include ***/
#include <iostream>
#include <filesystem>
#include <ryusei/common/logger.hpp>
#include <ryusei/common/defs.hpp>
#include <ryusei/common/math.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/optional.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#define _USE_MATH_DEFINES
#include <cmath>
#include <string>
#include <vector>
#include <array>
#include <opencv2/opencv.hpp>

using namespace project_ryusei;
namespace pr = project_ryusei;
#define _SILENCE_EXPERIMENTAL_FILESYSTEM_DEPRECATION_WARNING
namespace fs = std::filesystem;
namespace prop = boost::property_tree;

using namespace std;
using namespace cv;

static inline float Deg2Rad(float deg) {return static_cast<float>(deg * M_PI / 180.0);}
static inline float Rad2Deg(float rad) {return static_cast<float>(rad * 180 / M_PI);}

/*** 点群をカメラ画像の座標に合わせる(回転させる)関数のパラメータ ***/
double X_DIFF;
double Y_DIFF;
double Z_DIFF;
double ROLL;
double PITCH;
double YAW;

/*** file_img_pcd_name.iniのパス ***/
string IMG_PCD_PATH;
/*** streamcam.yamlのパス ***/
string CAM_YAML_PATH;

/*** 赤Mercury  3D-lidar : pandar_xt_32 ***/
double LIDAR_VIEW_ANGLE_H = Deg2Rad(140.0); // 障害物を探索する水平画角 /* 140 ~ 360° */
double LIDAR_VIEW_ANGLE_V = Deg2Rad(32.0); // 障害物を探索する垂直画角
double LIDAR_RESOLUTION_H = Deg2Rad(0.36); // LiDARの水平解像度(分解能)
double LIDAR_RESOLUTION_V = Deg2Rad(1.0); // LiDARの垂直解像度(分解能)

/*** 青Mercury  3D-lidar : pandar40 ***/
// double LIDAR_VIEW_ANGLE_H = Deg2Rad(140.0); // 障害物を探索する水平画角 /* 140 ~ 360° */
// double LIDAR_VIEW_ANGLE_V = Deg2Rad(40.0); // 障害物を探索する垂直画角
// double LIDAR_RESOLUTION_H = Deg2Rad(0.33); // LiDARの水平解像度(分解能)
// double LIDAR_RESOLUTION_V = Deg2Rad(1.0); // LiDARの垂直解像度(分解能)

/*** LiDAR画像に関するパラメータ ***/
double DETECT_HEIGHT_MIN; // 障害物の高さ(最小)
double DETECT_HEIGHT_MAX; // 障害物の高さ(最大)
double LIDAR_HEIGHT; // LiDARの取り付け高さ
double REFLECT_THRESH; // 反射強度の閾値
double MIN_RANGE; // 投影する距離の最小値
double MAX_RANGE; // 投影する距離の最大値
double MAX_REFLECT; // 投影する反射強度の最大値
int MIN_PIX_NUM_SIGN; // 標識と判定するピクセル数の最小
int MAX_PIX_NUM_SIGN; // 標識と判定するピクセル数の最大
double MIN_ASPECT_RATIO_SIGN; // 標識と判定するアスペクト比の最小
double MAX_ASPECT_RATIO_SIGN; // 標識と判定するアスペクト比の最大

/*** HSVの閾値 ***/
int MIN_H_RED_01;
int MAX_H_RED_01;
int MIN_H_RED_02;
int MAX_H_RED_02;
int MIN_S_RED;
int MAX_S_RED;
int MIN_V_RED;
int MAX_V_RED;

int MIN_H_GREEN;
int MAX_H_GREEN;
int MIN_S_GREEN;
int MAX_S_GREEN;
int MIN_V_GREEN;
int MAX_V_GREEN;

int MIN_H_YELLOW;
int MAX_H_YELLOW;
int MIN_S_YELLOW;
int MAX_S_YELLOW;
int MIN_V_YELLOW;
int MAX_V_YELLOW;

/*** 信号の候補領域のピクセル数の閾値 ***/
int MIN_PIX_NUM;
int MAX_PIX_NUM;
/*** 信号の候補領域のアスペクト比の閾値 ***/
/* 横 : 縦 = ASPECT_RATIO : 1 */
double MIN_ASPECT_RATIO;
double MAX_ASPECT_RATIO;
/*** 候補領域内の黄色画素ピクセル数の閾値 ***/
int YELLOW_PIX_TH;
double MIN_ASPECT_RATIO_YELLOW;
double MAX_ASPECT_RATIO_YELLOW;
/*** 信号認識領域の標識の矩形による倍率 ***/
float SIGNAL_RECO_REGION_WIDTH, SIGNAL_RECO_REGION_HEIGHT;

namespace signal_reco {

class SignalReco {
public:
    SignalReco();
    ~SignalReco();

    void initImgPcdName(const string &path);
    void initParam(const string &path);

    void getFiles(const fs::path &path, const string &extension, vector<fs::path> &files);
    bool loadPCD(const string &path, vector<LidarData> &points);
    void euler2Quaternion(float roll, float pitch, float yaw, float &q_w, float &q_x, float &q_y, float &q_z);
    void rotatePoints(const vector<LidarData> &src, float roll, float pitch, float yaw, vector<LidarData> &dst);
    void projectToImage(const vector<LidarData> &points, cv::Mat &lidar_img, bool is_for_reco = false);
    void projectToImageForView(const vector<LidarData> &points, Mat &lidar_img);
    void drawObjectsReflect(const Mat &lidar_data, Mat &img);
    void drawObjectsReflectForView(const Mat &lidar_data, Mat &img);
    void drawObjectsRange(const Mat &lidar_data, Mat &img);
    void drawObjectsRangeForView(const Mat &lidar_data, Mat &img);
    void rectangleReflect(const Mat &lidar_img_ref, Mat &lidar_img_ref_bin, vector<vector<cv::Point2i>> &sign_rects);
    void screen2CenteredCoords(cv::Size image_size, const vector<vector<cv::Point2i>> &sign_rect_refimg_screen, vector<vector<cv::Point2i>> &sign_rect_refimg_centered_screen);
    void centeredScreen2RobotCoords(const Mat &lidar_img, const vector<vector<cv::Point2i>> &sign_rects_refimg_screen, const vector<vector<cv::Point2i>> &sign_rect_refimg_centered_screen, vector<vector<cv::Point3f>> &sign_rect_points_robot);
    void rotateRectPoints(const std::vector<std::vector<cv::Point3f>> &src, float roll, float pitch, float yaw, std::vector<std::vector<cv::Point3f>> &dst);
    void perspectiveProjectionModel(const vector<vector<cv::Point3f>> &sign_rect_points_camera, vector<vector<cv::Point2i>> &sign_rects_perspective);
    void centeredScreen2ScreenCoords(cv::Size image_size, vector<vector<cv::Point2i>> &sign_rects_camimg_perspective, vector<vector<cv::Point2i>> &sign_rects);
    void drawSignOnCameraImg(const Mat &camera_img,  const vector<vector<cv::Point2i>> &sign_rects);
    void storeSignalRects(const vector<vector<cv::Point2i>> &sign_rects, vector<vector<cv::Point2i>> &signal_rects);
    void drawSignalRectsOnCameraImg(Mat &camera_img, const vector<vector<cv::Point2i>> &signal_rects);
    void cropSignalRectsFromCameraImg(const Mat &camera_img, const vector<vector<Point2i>> &signal_rects, vector<Mat> &signal_imgs);
    void rgb2HSV(const vector<Mat> &signal_imgs, vector<Mat> &signal_imgs_hsv);
    void extractColor(const vector<Mat> &signal_imgs_hsv, vector<Mat> &signal_imgs_extract_red, bool is_red);
    void medianImgs(const vector<Mat> &signal_imgs_extract, vector<Mat> &signal_imgs_extract_median);
    void dilateImgs(const vector<Mat> &signal_imgs_extract_median, vector<Mat> &signal_imgs_extract_dilated);
    void labeling(const vector<Mat> &imgs_extract_dilated, vector<Mat> &imgs_rabeled, vector<vector<Mat>> &stats);
    void drawSignalCandidates(const vector<Mat> &src, const vector<vector<Mat>> &stats, bool is_red = true);
    void extractYellow(vector<Mat> &signal_imgs, vector<vector<Mat>> &imgs_red_stats, vector<vector<vector<Mat>>> &imgs_extract_yellow);
    void labelingYellow(const std::vector<std::vector<std::vector<cv::Mat>>> &imgs_ex_yellow, 
        const std::vector<std::vector<cv::Mat>> &imgs_original_stats,std::vector<std::vector<cv::Mat>> &imgs_stats_valid,
        std::vector<std::vector<std::vector<cv::Mat>>> &imgs_yellow_labeling, std::vector<std::vector<std::vector<cv::Mat>>> &imgs_yellow_stats,
        int &num_figures, std::string &signal_state, bool is_red);
    void drawRects(const std::vector<std::vector<cv::Mat>> &stats, int num_figures, vector<cv::Mat> &img, bool is_red);
    void drawResult(Mat &camera_img, const string &signal_state);
    void loop_main();
    /*** 画像、PCD、 カメラのパラメータファイル　のパス ***/
    std::string img_path, pcd_path, cam_yaml_path;
    /*** 画像のファイルパス、PCDのファイルパス　配列 ***/
    vector<fs::path> files_png, files_pcd;
    /* 配列内のファイル番号 */
    int file_cnt = 0;
    /*** カメラキャリブレーションパラメータ ***/
    Mat camera_params, distortion_params, rectify_params, projection_params;
    Mat map_x, map_y;
    /*** カメラ ***/
    /* 歪み補正前カメラ画像 */
    Mat src_camera_img;
    /* 歪み補正後カメラ画像 */
    Mat camera_img;
    /*** LiDAR ***/
    /* 点群情報（反射強度, 距離）を格納するMat */
    Mat lidar_img;
    /* LiDAR画像 : 反射強度画像, 距離画像 */
    Mat lidar_img_ref, lidar_img_range;
    Mat lidar_img_ref_color;
    Mat lidar_img_ref_bin;
    /* LiDAR画像 : 反射強度画像, 距離画像（確認用） */
    Mat lidar_img_fov, lidar_img_range_fov, lidar_img_ref_fov;
    /* 回転補正後の点群 */
    vector<LidarData> points;
    /* カメラ座標系に変換した点群 */
    vector<cv::Point3f> points_cam_coords;
    /* 標識の矩形 */
    vector<vector<cv::Point2i>> sign_rects_refimg_screen; // 反射強度画像の標識の矩形(px)(スクリーン座標系)
    vector<vector<cv::Point2i>> sign_rects_refimg_centered_screen; // 反射強度画像の標識の矩形(px)(正規スクリーン座標系)
    vector<vector<cv::Point3f>> src_sign_rect_points_robot; // 標識の矩形(m)(ロボット座標系)
    vector<vector<cv::Point3f>> sign_rect_points_robot; // 標識の矩形(m)(ロボット座標系)
    vector<vector<cv::Point2i>> sign_rects_camimg_perspective; // カメラ画像の標識の矩形(m)(透視投影モデル適用後)
    vector<vector<cv::Point2i>> sign_rects; // カメラ画像の標識の矩形(px)(スクリーン座標系)
    /* 信号が存在する領域を切り抜いた矩形 */
    vector<vector<cv::Point2i>> signal_rects;
    /* 信号の画像 */
    vector<Mat> signal_imgs;
    vector<Mat> signal_imgs_hsv;
    vector<Mat> imgs_extract_red;
    vector<Mat> imgs_extract_green;
    vector<Mat> imgs_red_median;
    vector<Mat> imgs_green_median;
    vector<Mat> imgs_red_dilated;
    vector<Mat> imgs_green_dilated;
    /* 信号領域内のblob */
    vector<Mat> imgs_red_labeling;
    vector<Mat> imgs_green_labeling;
    vector<vector<Mat>> imgs_red_stats; // 赤色のblobの矩形情報
    vector<vector<Mat>> imgs_green_stats; // 緑色のblobの矩形情報
    vector<vector<vector<Mat>>> imgs_red_ex_yellow; // 赤色のblobの中から黄色の人人型を抽出
    vector<vector<vector<Mat>>> imgs_green_ex_yellow; // 緑色のblobの中から黄色の人型を抽出
    vector<vector<vector<Mat>>> imgs_red_ex_yellow_labeling;
    vector<vector<vector<Mat>>> imgs_green_ex_yellow_labeling;
    vector<vector<vector<Mat>>> imgs_red_ex_yellow_stats; // 赤色のblobの中から黄色の人型にラベリング処理を適用
    vector<vector<vector<Mat>>> imgs_green_ex_yellow_stats; // 緑色のblobの中から黄色の人型にラベリング処理を適用
    int num_figures_red = 0, num_figures_green = 0; // 黄色の人型の数を格納する変数
    string signal_state;
};
} // namespace signal_reco

#endif // TRAFFIC_SIGNAL_RECO_HPP_
