#声明要求的 cmake 最低版本
cmake_minimum_required( VERSION 2.8 )

# 添加 c++11 标准支持
set( CMAKE_CXX_FLAGS "-std=c++11" )


# 设定二进制文件路径
SET( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
# 设定头文件路径
INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/include )
# 设定库文件编译路径
SET( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )
# 设定 .cmake 文件存放路径
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# Sophus
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

# opencv
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# g2o
find_package( G2O REQUIRED )
IF( G2O_FOUND )
    include_directories( ${G2O_INCLUDE_DIR} )
    message( "G2O lib is found:" ${G2O_INCLUDE_DIR} )
ENDIF( G2O_FOUND )

find_package( Eigen3 REQUIRED )
find_package( CSparse REQUIRED )
include_directories( ${CSPARSE_INCLUDE_DIR} )
include_directories( ${EIGEN3_INCLUDE_DIR} )

include_directories( ${PROJECT_SOURCE_DIR}/ceres )

#add_executable( image_basics image_basics.cpp )
#target_link_libraries( image_basics ${OpenCV_LIBS} )

#add_executable( test_solve_pnp testSolvePnp.cpp)
#target_link_libraries( test_solve_pnp ${OpenCV_LIBS} )

#add_executable( opencv_viz_basic opencv_viz_basic.cpp )
#target_link_libraries( opencv_viz_basic ${OpenCV_LIBS} )

add_executable( opencv_kalmanfilter ${PROJECT_SOURCE_DIR}/src/opencv_kalmanfilter.cpp )
target_link_libraries( opencv_kalmanfilter ${OpenCV_LIBS} )

add_executable( deepar_g2o_kalmanfilter ${PROJECT_SOURCE_DIR}/src/deepar_g2o_kalmanfilter.cpp )
target_link_libraries( deepar_g2o_kalmanfilter ${OpenCV_LIBS} )
