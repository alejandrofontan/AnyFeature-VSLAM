cmake_minimum_required(VERSION 3.16)
project(AnyFeature-VSLAM)

# Compiler settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE Release)

# Output directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# Compiler flags
add_compile_options(-Wall -Wextra -Wpedantic -O3 -march=native)

set(SRCS_ANYFEATURE
        src/System.cc
        src/Tracking.cc
        src/LocalMapping.cc
        src/LoopClosing.cc
        src/ORBextractor.cc
        src/FeatureMatcher.cc
        src/FrameDrawer.cc
        src/Converter.cc
        src/MapPoint.cc
        src/KeyFrame.cc
        src/Map.cc
        src/MapDrawer.cc
        src/Optimizer.cc
        src/PnPsolver.cc
        src/Frame.cc
        src/KeyFrameDatabase.cc
        src/Sim3Solver.cc
        src/Initializer.cc
        src/Viewer.cc
        src/MathFunctions.cpp
        src/Observation.cpp
        src/FeatureExtractor.cpp
        src/Utils.cpp
        src/Vocabulary.cpp
        src/Image.cpp
        src/Feature_orb32.cpp
        src/Feature_brisk48.cpp
        src/Feature_akaze61.cpp
        src/Feature_surf64.cpp
        src/Feature_kaze64.cpp
        src/Feature_sift128.cpp
        src/Feature_r2d2_128.cpp
        src/Feature_anyFeatBin.cpp
        src/Feature_anyFeatNonBin.cpp
) 

# Dependencies
find_package(OpenCV REQUIRED )
find_package(Eigen3  REQUIRED NO_MODULE)
find_package(Pangolin REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(g2o REQUIRED)
find_package(Python REQUIRED COMPONENTS Interpreter)
find_package(brisk REQUIRED)
find_package(akaze REQUIRED)
find_package(SiftGPU REQUIRED)

set(DBoW2_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/include")
set(DBoW2_LIBRARIES "${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so")

add_library(${PROJECT_NAME} SHARED ${SRCS_ANYFEATURE})

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${DBoW2_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBRARIES}
        Eigen3::Eigen
        ${YAML_CPP_LIBRARIES}
        ${g2o_LIBRARIES}
        ${DBoW2_LIBRARIES}
        ${brisk_LIBRARIES}
        ${akaze_LIBRARIES}
        ${SiftGPU_LIBRARIES}
        ${Pangolin_LIBRARIES}
)

# Example executables
add_executable(vslamlab_anyfeature_mono src/vslamlab_anyfeature_mono.cpp)
target_link_libraries(vslamlab_anyfeature_mono ${PROJECT_NAME})

# add_executable(createVocabulary src/createVocabulary.cpp)
# target_link_libraries(createVocabulary ${PROJECT_NAME})

# Install targets and headers
install(TARGETS ${PROJECT_NAME} vslamlab_anyfeature_mono
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(DIRECTORY include/ DESTINATION include)