# set output dir for test results
set(rosbuild_test_results_dir ${CMAKE_CURRENT_SOURCE_DIR})

rosbuild_add_gtest(testfitting_cpp testfitting.cpp)
target_link_libraries(testfitting_cpp fitting)

# テストを走らせるコード
#rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/opencv_fitting_test.xml)
