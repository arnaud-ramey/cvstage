/*!
  \file        draw_xy_lines.h
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

#ifndef cvstage_PLUGINS_XY_LINES_H
#define cvstage_PLUGINS_XY_LINES_H

#include "cvstage/cvstage.h"

namespace cvstage_plugins {

/*!
  \return the distance AB
  */
template<class Point2_A, class Point2_B>
static inline double
distance_points(const Point2_A & A, const Point2_B & B) {
  return hypot(A.x - B.x, A.y - B.y);
}

////////////////////////////////////////////////////////////////////////////////

/*!
 * print a point in a fancy way
 * @param p
 * @return the fancy std::string
 */
template<class Point2>
static inline std::string printP2(const Point2 & p) {
  std::ostringstream ans;
  ans << "(" << p.x << ", " << p.y << ")";
  return ans.str();
}

////////////////////////////////////////////////////////////////////////////////

template<class T1, class T2>
void plot_xy_lines(MiniStage & ms,
                   const std::vector<T1> & xvec,
                   const std::vector<T2> & yvec,
                   const cv::Scalar& color, int thickness=1, int lineType=8) {
  unsigned int npts = std::min(xvec.size(), yvec.size());
  if (npts < 1)
    return;
  cv::Point curr, prev = ms.world2pixel(xvec[0], yvec[0]);
  for (unsigned int pt_idx = 1; pt_idx < npts; ++pt_idx) {
    curr = ms.world2pixel(xvec[pt_idx], yvec[pt_idx]);
    cv::line(ms.get_viz(), prev, curr, color, thickness, lineType);
    prev = curr;
  } // end loop pt_idx
}

template<class Pt2>
void plot_xy_lines(MiniStage & ms,
                   const std::vector<Pt2> & vec,
                   const cv::Scalar& color, int thickness=1, int lineType=8) {
  unsigned int npts = vec.size();
  if (npts < 1)
    return;
  cv::Point curr, prev = ms.world2pixel(vec[0].x, vec[0].y);
  for (unsigned int pt_idx = 1; pt_idx < npts; ++pt_idx) {
    curr = ms.world2pixel(vec[pt_idx]);
    cv::line(ms.get_viz(), prev, curr, color, thickness, lineType);
    prev = curr;
  } // end loop pt_idx
}

////////////////////////////////////////////////////////////////////////////////

template<class T1, class T2>
void plot_xy_pts(MiniStage & ms,
                 const std::vector<T1> & xvec,
                 const std::vector<T2> & yvec,
                 const cv::Scalar& color, int thickness=1, int lineType=8,
                 const double xshift = 0, const double yshift = 0) {
  unsigned int npts = std::min(xvec.size(), yvec.size());
  for (unsigned int pt_idx = 1; pt_idx < npts; ++pt_idx)
    cv::circle(ms.get_viz(), ms.world2pixel(xvec[pt_idx]+xshift, yvec[pt_idx]+yshift),
               thickness, color, -1, lineType);
}

template<class Pt2>
void plot_xy_pts(MiniStage & ms,
                 const std::vector<Pt2> & vec,
                 const cv::Scalar& color, int thickness=1, int lineType=8,
                 const double xshift = 0, const double yshift = 0) {
  unsigned int npts = vec.size();
  for (unsigned int pt_idx = 1; pt_idx < npts; ++pt_idx)
    cv::circle(ms.get_viz(), ms.world2pixel(vec[pt_idx].x+xshift, vec[pt_idx].y+yshift),
               thickness, color, -1, lineType);
}

} // end namespace cvstage_plugins

#endif // cvstage_PLUGINS_XY_LINES_H
