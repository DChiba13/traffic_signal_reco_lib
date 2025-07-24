#include "traffic_signal_reco.hpp"
/*** Global variable ***/
static signal_reco::SignalReco signal;
using namespace signal_reco;
int main() {
  /*** ディレクトリからpngとpcdファイル名を取得 ***/
  signal.getFiles(signal.img_path, ".png", signal.files_png);
  signal.getFiles(signal.pcd_path, ".pcd", signal.files_pcd);

  while(1) 
  {
    cout << "Camera file : " << signal.files_png[signal.file_cnt].string() << endl;
    /*** カメラ画像の読み込み ***/
    signal.src_camera_img = imread(signal.files_png[signal.file_cnt].string(), 1);
    /*** 点群の読み込み ***/
    signal.loadPCD(signal.files_pcd[signal.file_cnt].string(), signal.src_points);
    signal.loop_main();
    imshow("Window Main", signal.camera_img);
    // imshow("LiDAR Reflect Image", signal.lidar_img_ref);
    // imshow("LiDAR Reflect Image For View", signal.lidar_img_ref_fov);
    // imshow("LiDAR Range Image For View", signal.lidar_img_range_fov);
    // imshow("LIDAR Reflect Img Bin", signal.lidar_img_ref_bin);
    // imshow("LiDAR Range Image", signal.lidar_img_range);
    // signal_imgs, signal_img_extract_red, greenの画像表示
    for (int i = 0; i < signal.signal_imgs.size(); i++) {
      imshow("Signal Image " + to_string(i), signal.signal_imgs[i]);
      imshow("Red " + to_string(i), signal.imgs_extract_red[i]);
      imshow("Green " + to_string(i), signal.imgs_extract_green[i]);
      // imshow("RMedian " + to_string(i), signal.imgs_red_median[i]);
      // imshow("GMedian " + to_string(i), signal.imgs_green_median[i]);
      // imshow("RDilated " + to_string(i), signal.imgs_red_dilated[i]);
      // imshow("GDilated " + to_string(i), signal.imgs_green_dilated[i]);
    }
    // imgs_ex_yellowの画像表示
    for (int i = 0; i < signal.imgs_red_ex_yellow.size(); i++) {
      for (int j = 0; j < signal.imgs_red_ex_yellow[i].size(); j++) {
        for(int k = 0; k < signal.imgs_red_ex_yellow[i][j].size(); k++) {
          imshow("RYellow " + to_string(i) + "_" + to_string(j) + "_" + to_string(k), signal.imgs_red_ex_yellow[i][j][k]);
        }
      }
    }
    for (int i = 0; i < signal.imgs_green_ex_yellow.size(); i++) {
      for (int j = 0; j < signal.imgs_green_ex_yellow[i].size(); j++) {
        for(int k = 0; k < signal.imgs_green_ex_yellow[i][j].size(); k++) {
          imshow("GYellow " + to_string(i) + "_" + to_string(j) + "_" + to_string(k), signal.imgs_green_ex_yellow[i][j][k]);
        }
      }
    }
    cout << "Signal state : " << signal.signal_state << endl;
    int key = waitKey(0);
    if(key == ' ') break;
    else if(key == 'a') --signal.file_cnt;
    else if(key == 'A') signal.file_cnt -= 10;
    else if(key == 'D') signal.file_cnt += 10;
    else if(key == 'd') ++signal.file_cnt;
    int sz = signal.files_png.size();
    if(signal.file_cnt > sz - 1) signal.file_cnt = 0;
    else if (signal.file_cnt < 0) signal.file_cnt = sz - 1;
    destroyAllWindows();
  } // while(1)
  return 0;
} // main()