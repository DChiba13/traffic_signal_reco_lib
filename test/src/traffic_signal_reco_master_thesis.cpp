#include "traffic_signal_reco.hpp"
#include <chrono>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <ctime>
#include <vector>
#include <string>

using namespace signal_reco;
namespace fs = std::filesystem;

/*** Global variable ***/
static SignalReco signal;

int main() {
    /*** 入力ファイル一覧取得 ***/
    signal.getFiles(signal.img_path, ".png", signal.files_png);
    signal.getFiles(signal.pcd_path, ".pcd", signal.files_pcd);

    /*** ベース保存ディレクトリ ***/
    // std::string base_dir = "/home/chiba/share/result_tr_reco/master_thesis";
    // std::string base_dir = "/home/revast/chiba/camelidar_inference_results/red_results";
    // std::string base_dir = "/home/revast/chiba/camelidar_inference_results/blue1_results";
    // std::string base_dir = "/home/revast/chiba/camelidar_inference_results/blue2_results";
    std::string base_dir = "/home/revast/chiba/camelidar_inference_results/blue3_results";

    if (!fs::exists(base_dir)) fs::create_directories(base_dir);

    /*** 日時文字列作成（YYYY_MM_DD_HH_MM） ***/
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm local_tm = *std::localtime(&now_c);

    std::ostringstream oss;
    oss << std::put_time(&local_tm, "%Y_%m_%d_%H_%M");
    std::string datetime_str = oss.str();

    /*** 日時フォルダ作成 ***/
    std::string save_dir = (fs::path(base_dir) / datetime_str).string();
    fs::create_directories(save_dir);

    /*** CSV ファイルパス ***/
    std::string csv_path = (fs::path(save_dir) / (datetime_str + ".csv")).string();

    /*** CSV オープン ***/
    std::ofstream csv(csv_path);
    if (!csv.is_open()) {
        std::cerr << "Failed to open CSV file: " << csv_path << std::endl;
        return -1;
    }
    csv << "image_name,time_ms,signal_state,result\n";

    std::cout << "-------------------------------------------\n";
    std::cout << " Master Thesis Auto Execution Mode\n";
    std::cout << "-------------------------------------------\n";
    std::cout << " Saving into directory: " << save_dir << "\n\n";

    int total_files = signal.files_png.size();
    signal.file_cnt = 0;

    /*** 自動処理ループ ***/
    while (signal.file_cnt < total_files) {
        fs::path png_path = signal.files_png[signal.file_cnt];
        fs::path pcd_path = signal.files_pcd[signal.file_cnt];

        std::cout << "[" << signal.file_cnt + 1 << "/" << total_files << "] "
                  << png_path.filename().string() << "\n";

        /*** 読み込み ***/
        signal.src_camera_img = cv::imread(png_path.string(), 1);
        signal.loadPCD(pcd_path.string(), signal.points);

        if (signal.src_camera_img.empty()) {
            std::cerr << "Failed to load image: " << png_path << std::endl;
            signal.file_cnt++;
            continue;
        }

        /*** 認識処理 ***/
        auto start = std::chrono::high_resolution_clock::now();
        signal.loop_main();
        auto end = std::chrono::high_resolution_clock::now();
        double duration = std::chrono::duration<double, std::milli>(end - start).count();

        std::string img_name = png_path.filename().string();

        /*** CSV書き込み（Correct固定） ***/
        csv << img_name << ","
            << std::fixed << std::setprecision(1) << duration << ","
            << signal.signal_state << ","
            << "Correct"
            << "\n";

        /*** 結果画像保存 ***/
        std::string save_img = (fs::path(save_dir) / img_name).string();
        cv::imwrite(save_img, signal.camera_img);

        std::cout << "  → Image saved: " << save_img << "\n";

        signal.file_cnt++;
    }

    csv.close();

    std::cout << "\n-------------------------------------------\n";
    std::cout << " Processing complete.\n";
    std::cout << " Results saved in: " << save_dir << "\n";
    std::cout << "-------------------------------------------\n";

    return 0;
}
