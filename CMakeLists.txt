cmake_minimum_required(VERSION 3.1)
project(XUtilsLib VERSION 0.1 LANGUAGES CXX)

find_package(Eigen3 3.3 REQUIRED)
find_package(OpenCV 3.4 REQUIRED)

add_library(XUtils SHARED "")

target_sources(XUtils
PRIVATE
    src/xutils/IOWrapper/rgbd_camera.cpp
    src/xutils/IOWrapper/tum_loader.cpp
)

target_include_directories(XUtils
PUBLIC
    $<INSTALL_INTERFACE:include>   
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_compile_features(XUtils 
PRIVATE 
    cxx_std_11
)

target_link_directories(XUtils
PRIVATE
    Eigen3::Eigen
    /usr/lib/libOpenNI2.so
    ${OpenCV_LIBRARIES}
)

install(TARGETS XUtils
        EXPORT XUtils-targets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        CONFIGURATIONS Release)

install(EXPORT XUtils-targets
        NAMESPACE XUtils::
        FILE XUtilsConfig.cmake
        DESTINATION lib/cmake/XUtils)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include
        DESTINATION ${CMAKE_INSTALL_PREFIX})