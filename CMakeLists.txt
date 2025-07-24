cmake_minimum_required(VERSION 3.0)
project(traffic_signal_reco_lib)

# Include common headers globally（lib/include）
include_directories(lib/include)

# サブディレクトリ追加
add_subdirectory(lib)
add_subdirectory(test)