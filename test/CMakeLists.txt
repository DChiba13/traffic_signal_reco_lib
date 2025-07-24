cmake_minimum_required(VERSION 3.0)
project(traffic_signal_reco_test)

# ビルドタイプとC++17設定
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O0")

# 依存ライブラリ
find_package(OpenCV REQUIRED core imgproc highgui calib3d)
find_package(Boost REQUIRED system)

set(HEADER_DIR "" CACHE PATH "Path to headers")
set(LIBRARY_DIR "" CACHE PATH "Path to libraries")

# インクルードディレクトリ
include_directories(
  ../lib/include
  $ENV{C_INCLUDE_PATH}
  $ENV{CPLUS_INCLUDE_PATH}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${HEADER_DIR}
)
 # リンクディレクトリ
link_directories(
  $ENV{LIBRARY_PATH}
  ${OpenCV_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
  ${LIBRARY_DIR}
)
set(DEPENDENT_LIBS ${OpenCV_LIBS};traffic_signal_reco_core rs_common)
set(EXECUTABLES traffic_signal_reco_test)

foreach(target IN LISTS EXECUTABLES)
  message("${target}")
  add_executable(${target} src/${target}.cpp)
  target_link_libraries(${target} ${DEPENDENT_LIBS})
endforeach()

# Set installation
foreach(target IN LISTS EXECUTABLES)
  install(
    TARGETS ${target}
    RUNTIME DESTINATION bin
  )
endforeach()
