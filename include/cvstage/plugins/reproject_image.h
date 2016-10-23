/*!
  \file        reproject_image.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/3/10

________________________________________________________________________________

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
________________________________________________________________________________

Some plugins to draw more stuff with a MiniStage.

 */

#ifndef cvstage_PLUGINS_REPROJECT_IMAGE_H
#define cvstage_PLUGINS_REPROJECT_IMAGE_H

#include "cvstage/cvstage.h"
#include "image_geometry/pinhole_camera_model.h"

namespace cvstage_plugins {

static const double NAN_DOUBLE = 0;

//! \return \true if distance is a NAN (or a simulated one)
inline bool is_nan_depth(const float & distance) {
  return (distance == 0 || isnan(distance));
}

////////////////////////////////////////////////////////////////////////////////

/*!
 * Template version of pixel2world_depth().
 * It calls the non templated version, with cv::Point3d,
 * then converts it.
 */
template<class Pt3>
inline Pt3 pixel2world_depth
(const cv::Point2d & depth_pt,
 const image_geometry::PinholeCameraModel & depth_cam_model,
 const double & depth_value)
{
  cv::Point3d ans_cv =
      pixel2world_depth<cv::Point3d>(depth_pt, depth_cam_model, depth_value);
  Pt3 ans;
  ans.x = ans_cv.x;
  ans.y = ans_cv.y;
  ans.z = ans_cv.z;
  return ans;
} // end pixel2world_depth()

////////////////////////////////////////////////////////////////////////////////

/*!
 * The lowest level version, needing the most data.
 * \arg pt_rgb the point in the 2D RGB image, classic image frame
     +--------> x
     |
     |
   y V

   \return pt3d with such (z in direction of the scene):

<scene here>

   ^ z
    \
     +---> x
     |
     |
   y V

 */
template<>
inline cv::Point3d pixel2world_depth
(const cv::Point2d & depth_pt,
 const image_geometry::PinholeCameraModel & depth_cam_model,
 const double & depth_value)
{
  if (cvstage_plugins::is_nan_depth(depth_value))
    return cv::Point3d(NAN_DOUBLE, NAN_DOUBLE, NAN_DOUBLE);
  cv::Point3d line_vec = depth_cam_model.projectPixelTo3dRay(depth_pt);
  // double line_vec_norm = geometry_utils::norm(line_vec);
  //  if (fabs(line_vec_norm) < 1E-5)
  //    return cv::Point3d(0, 0, 0);
  //    printf("depth_pt%s, line_vec:%s, line_vec_norm:%g",
  //             geometry_utils::printP2(depth_pt).c_str(),
  //             geometry_utils::printP(line_vec).c_str(),
  //             line_vec_norm);
  // return line_vec * (depth_value / line_vec_norm);

  if (fabs(line_vec.z) < 1E-5)
    return cv::Point3d(0, 0, 0);
  // return line_vec * (depth_value / line_vec.z);
  return cv::Point3d(line_vec.x * depth_value / line_vec.z,
                     line_vec.y * depth_value / line_vec.z,
                     depth_value);
} // end pixel2world_depth()

////////////////////////////////////////////////////////////////////////////////

/*!
 * \brief pixel2world_rgb_color255
 * \param bgr_img
 *   The color image
 * \param depth_img
 *   The depth image
 * \param depth_cam_model
 *   The camera model of the depth image
 * \param depth_reprojected
 *   [OUT] the 3D points
 * \param colors
 *   [OUT] a vector, of, for instance, cv::Scalar or cv::Vec3b
 * \param data_step
 *   The increment in columns and rows for reprojecting the pixels.
 *   Use 1 to reproject all pixels, 2 to reproject 50% of them, etc.
 * \param mask_depth
 *   An optional mask to indicate what points should be kept
 *  (0=> discared pixel, anything else => reproject)
 * \param remove_nans
 *    If true, do not insert into the cloud points that correspond to
 *    an undefined depth (shiny surfaces, etc.)
 * \return
 *    true if success
 */
template<class Pt3, class Color255>
inline bool pixel2world_rgb_color255
(const cv::Mat & bgr_img,
 const cv::Mat & depth_img,
 const image_geometry::PinholeCameraModel & depth_cam_model,
 std::vector<Pt3> & depth_reprojected,
 std::vector<Color255> & colors,
 const unsigned int data_step = 1,
 const cv::Mat1b & mask_depth = cv::Mat(),
 bool remove_nans = false)
{
  // clear answer
  int cols = depth_img.cols, rows = depth_img.rows;
  int n_pts = rows * cols / (data_step * data_step);
  depth_reprojected.clear();
  depth_reprojected.reserve(n_pts);
  colors.clear();
  colors.reserve(n_pts);

  // dimensions check
  if (depth_img.size() != bgr_img.size()) {
    printf("pixel2world_rgb(): depth size (%i, %i) != bgr_img size (%i, %i)\n",
           depth_img.cols, depth_img.rows, bgr_img.cols, bgr_img.rows);
    return false;
  }
  bool use_mask = (!mask_depth.empty());
  if (use_mask && depth_img.size() != mask_depth.size()) {
    printf("pixel2world_rgb(): depth size (%i, %i) != mask_depth size (%i, %i) -> "
           "no using mask\n", depth_img.cols, depth_img.rows,
           mask_depth.cols, mask_depth.rows);
    use_mask = false;
  }
  if (!depth_cam_model.initialized()) {
    printf("pixel2world_rgb(): depth_cam_model not initialized\n");
    return false;
  }
  if (depth_img.empty()) {
    printf("pixel2world_rgb(): empty images\n");
    return true;
  }

  // locates matrix header within a parent matrix
  // cf http://opencv.willowgarage.com/documentation/cpp/core_basic_structures.html
  cv::Size size;
  cv::Point offset;
  bgr_img.locateROI(size, offset);

  const uchar* mask_ptr;
  for (int row = 0; row < rows; row += data_step) {
    // get the address of row
    const float* depth_ptr = depth_img.ptr<float>(row);
    const cv::Vec3b* bgr_data = bgr_img.ptr<cv::Vec3b>(row);
    if (use_mask)
      mask_ptr = mask_depth.ptr<uchar>(row);
    for (int col = 0; col < cols; col += data_step) {
      if (use_mask && mask_ptr[col] == 0)
        continue;
      if (remove_nans && cvstage_plugins::is_nan_depth(depth_ptr[col]))
        continue;
      depth_reprojected.push_back
          ( pixel2world_depth<Pt3>
            (cv::Point2d(col + offset.x, row + offset.y),
             depth_cam_model, depth_ptr[col]) );
      //colors.push_back(color_utils::uchar_bgr_color_to_ros_rgba_color(bgr_data[col][0], bgr_data[col][1], bgr_data[col][2]));
      colors.push_back(Color255(bgr_data[col][0], bgr_data[col][1], bgr_data[col][2]));
    } // end loop col
  } // end loop row
  return true;
} // end pixel2world_rgb_color255()

////////////////////////////////////////////////////////////////////////////////

template<class Pt3>
void reproject_image(MiniStage & ms,
                     const cv::Mat & bgr_img,
                     const cv::Mat & depth_img,
                     const image_geometry::PinholeCameraModel & depth_cam_model,
                     std::vector<Pt3> & depth_reprojected,
                     std::vector<cv::Scalar> & colors,
                     const int data_step = 1,
                     const cv::Mat1b & mask_depth = cv::Mat())
{
  pixel2world_rgb_color255(bgr_img, depth_img, depth_cam_model,
                                                depth_reprojected, colors,
                                                data_step, mask_depth);
  unsigned int npts = colors.size();
  if (depth_reprojected.size() != npts) {
    printf("reproject_image(): depth_reprojected.size()=%li != colors.size()=%li!",
                depth_reprojected.size(), colors.size());
    return;
  }
  for (unsigned int pt_idx = 0; pt_idx < npts; ++pt_idx) {
    cv::circle(ms.get_viz(), ms.world2pixel(depth_reprojected[pt_idx]), 2,
               colors[pt_idx], -1);
  } // end loop pt_idx
} // end reproject_image();

} // end namespace cvstage_plugins

#endif // cvstage_PLUGINS_REPROJECT_IMAGE_H
