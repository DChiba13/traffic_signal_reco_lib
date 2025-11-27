#include "traffic_signal_reco.hpp"
#include <chrono>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <ctime>
#include <vector>
#include <string>

/*** Global variable ***/
static signal_reco::SignalReco signal;
using namespace signal_reco;
namespace fs = std::filesystem;

int main() {
  /*** ディレクトリからpngとpcdファイル名を取得 ***/
  signal.getFiles(signal.img_path, ".png", signal.files_png);
  signal.getFiles(signal.pcd_path, ".pcd", signal.files_pcd);

  /*** 保存ディレクトリ設定 ***/
  std::string base_dir = "/home/chiba/share/result_tr_reco/loop_time";
  if (!fs::exists(base_dir)) fs::create_directories(base_dir);

  /*** 日時付きファイル名生成 ***/
  auto now = std::chrono::system_clock::now();
  std::time_t now_c = std::chrono::system_clock::to_time_t(now);
  std::tm local_tm = *std::localtime(&now_c);

  std::ostringstream oss;
  oss << std::put_time(&local_tm, "%Y_%m_%d_%H_%M") << ".csv";
  std::string filename = (fs::path(base_dir) / oss.str()).string();

  /*** CSV出力用ファイルを開く ***/
  std::ofstream csv_file(filename);
  if (!csv_file.is_open()) {
    std::cerr << "Failed to open CSV file: " << filename << std::endl;
    return -1;
  }

  csv_file << "image_name,time_ms,signal_state,result\n";

  int loop_index = 0;

  /* 履歴管理（取り消し用）*/
  struct Record {
    std::string image_name;
    long time_ms;
    std::string signal_state;
    std::string result;
  };
  std::vector<Record> records;

  while (1)
  {
    std::cout << "Camera file : " << signal.files_png[signal.file_cnt].string() << std::endl;

    /*** カメラ画像の読み込み ***/
    signal.src_camera_img = imread(signal.files_png[signal.file_cnt].string(), 1);

    /*** 点群の読み込み ***/
    signal.loadPCD(signal.files_pcd[signal.file_cnt].string(), signal.points);

    /* 画像名を取得（例: 000123.png） */
    std::string img_name = signal.files_png[signal.file_cnt].filename().string();

    auto start = std::chrono::high_resolution_clock::now(); /* 開始時間 */
    signal.loop_main();
    auto end = std::chrono::high_resolution_clock::now();  /* 終了時間 */
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    imshow("Window Main", signal.camera_img);
    imshow("LiDAR Reflect Image", signal.lidar_img_ref);

    for (int i = 0; i < signal.signal_imgs.size(); i++) {
      imshow("Signal Image " + to_string(i), signal.signal_imgs[i]);
      imshow("Red " + to_string(i), signal.imgs_extract_red[i]);
      imshow("Green " + to_string(i), signal.imgs_extract_green[i]);
      imshow("RMedian " + to_string(i), signal.imgs_red_median[i]);
      imshow("GMedian " + to_string(i), signal.imgs_green_median[i]);
      imshow("RDilated " + to_string(i), signal.imgs_red_dilated[i]);
      imshow("GDilated " + to_string(i), signal.imgs_green_dilated[i]);
    }

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

    std::cout << "Signal state : " << signal.signal_state << std::endl;

    while (true) {
      int key = waitKey(0);
      std::string result_text;

      if (key == ' ') {
        std::cout << "Exiting..." << std::endl;
        csv_file.close();
        std::cout << "CSV file saved to: " << filename << std::endl;
        return 0;
      }
      else if (key == 'a') {
        /* Undo: 直前の結果を取り消す */
        if (!records.empty()) {
          records.pop_back();

          /* CSVを上書きし直す */
          csv_file.close();
          std::ofstream new_csv(filename);
          new_csv << "image_name,time_ms,signal_state,result\n";

          for (const auto &r : records) {
            new_csv << r.image_name << "," << r.time_ms << "," 
                    << r.signal_state << "," << r.result << "\n";
          }
          new_csv.close();

          if (loop_index > 0) loop_index--;
          if (signal.file_cnt > 0) {
            signal.file_cnt--;
            std::cout << "[UNDO] Removed last record. Back to previous file." << std::endl;
          } else {
            std::cout << "[WARN] Already at first file. Cannot go back further." << std::endl;
          }
        } else {
          std::cout << "[WARN] No record to undo." << std::endl;
        }
        break; /* 再表示 */
      }
      else if (key == 'd') {
        result_text = "Correct";
      }
      else if (key == 'e') {
        result_text = "Undetected";
      }
      else if (key == 'c') {
        result_text = "Incorrect";
      }
      else {
        std::cout << "[INFO] Invalid key. Try again." << std::endl;
        continue;
      }

      Record rec{img_name, duration, signal.signal_state, result_text};

      records.push_back(rec);

      csv_file << rec.image_name << "," << rec.time_ms << ","
               << rec.signal_state << "," << rec.result << "\n";
      csv_file.flush();

      loop_index++;
      signal.file_cnt++;
      int sz = signal.files_png.size();
      if (signal.file_cnt > sz - 1) signal.file_cnt = 0;
      else if (signal.file_cnt < 0) signal.file_cnt = sz - 1;
      break; /* 次の画像へ */
    }
  }

  return 0;
}
