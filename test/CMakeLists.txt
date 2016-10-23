catkin_add_gtest     (gtest_ellipse_utils
  gtest_ellipse_utils.cpp
  ../include/cvstage/plugins/draw_ellipses.h)
TARGET_LINK_LIBRARIES(gtest_ellipse_utils  ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_mini_stage  gtest_mini_stage.cpp)
TARGET_LINK_LIBRARIES(gtest_mini_stage  ${catkin_LIBRARIES})
