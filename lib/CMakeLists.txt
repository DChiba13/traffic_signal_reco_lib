cmake_minimum_required(VERSION 3.10)
project(traffic_signal_reco_core)

# C++17を有効化
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Debug用設定（
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O0")

# Build Type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build" FORCE)
endif()

# 依存ライブラリ
find_package(OpenCV REQUIRED core imgproc highgui flann calib3d)
find_package(Boost REQUIRED system)

# オプションの依存パス
set(HEADER_DIR "" CACHE PATH "Path to external headers")
set(LIBRARY_DIR "" CACHE PATH "Path to external libraries")

# includeパス
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  $ENV{C_INCLUDE_PATH}
  $ENV{CPLUS_INCLUDE_PATH}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${HEADER_DIR}
)

# linkディレクトリ
link_directories(
  $ENV{LIBRARY_PATH}
  ${OpenCV_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
  ${LIBRARY_DIR}
)

# 使用するライブラリ
set(DEPENDENT_LIBS ${OpenCV_LIBS} rs_common)

# ライブラリ作成
add_library(${PROJECT_NAME} SHARED
  src/traffic_signal_reco.cpp
)

# リンク
target_link_libraries(${PROJECT_NAME} ${DEPENDENT_LIBS})

# インストール
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)
