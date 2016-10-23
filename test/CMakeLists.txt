catkin_add_gtest     (gtest_ellipse_utils
  gtest_ellipse_utils.cpp
  ../include/cvstage/plugins/draw_ellipses.h)
TARGET_LINK_LIBRARIES(gtest_ellipse_utils  ${catkin_LIBRARIES})

catkin_add_gtest     (gtest_cvstage  gtest_cvstage.cpp)
TARGET_LINK_LIBRARIES(gtest_cvstage  ${catkin_LIBRARIES})
