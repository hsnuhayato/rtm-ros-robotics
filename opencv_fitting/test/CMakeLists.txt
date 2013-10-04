# 単体テスト
rosbuild_add_gtest(testfitting_unit testfitting_unit.cpp)
target_link_libraries(testfitting_unit fitting)

# 結合テスト
rosbuild_add_executable(testfitting_ros testfitting_ros.cpp)
target_link_libraries(testfitting_ros fitting)
rosbuild_add_gtest_build_flags(testfitting_ros)
#rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/opencv_fitting_test.xml)
