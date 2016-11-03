/*!
  \file        draw_ellipses.h
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

#ifndef cvstage_PLUGINS_costmap_H
#define cvstage_PLUGINS_costmap_H

#include "cvstage/cvstage.h"
#include <nav_msgs/GridCells.h>

namespace cvstage_plugins {

/*!
 * \brief costmap_to_polygon_list
 * \param costmap
 * \param out
 *    points in this order:
 *  y ^
 *    |  3   2
 *    |    x
 *    |  0   1
 *   0+---------> x
 */
template<class _Pt3>
void costmap_to_polygon_list(const nav_msgs::GridCells & costmap,
                             std::vector<_Pt3> & out) {
  double halfwidth = costmap.cell_width / 2;
  double halfheight = costmap.cell_height / 2;
  out.clear();
  out.reserve(costmap.cells.size() * 4);
  _Pt3 corner;
  for (unsigned int cell_idx = 0; cell_idx < costmap.cells.size(); ++cell_idx) {
    // A B
    // C D
    // add C
    corner.x = costmap.cells[cell_idx].x - halfwidth;
    corner.y = costmap.cells[cell_idx].y - halfheight;
    corner.z = costmap.cells[cell_idx].z;
    out.push_back(corner);
    // add D
    corner.x = costmap.cells[cell_idx].x + halfwidth;
    out.push_back(corner);
    // add B
    corner.y = costmap.cells[cell_idx].y + halfheight;
    out.push_back(corner);
    // add A
    corner.x = costmap.cells[cell_idx].x - halfwidth;
    out.push_back(corner);
  } // end loop cell_idx
} // end costmap_to_string()

////////////////////////////////////////////////////////////////////////////////

//! draw costmap
void draw_costmap(MiniStage & ms,
                  const nav_msgs::GridCells & map,
                  std::vector<cv::Point3f> & map_to_corners,
                  const cv::Scalar inner_color = CV_RGB(200, 200, 255),
                  const cv::Scalar edge_color = CV_RGB(0, 0, 255),
                  const int edge_width = 3) {
  // get corners of the cell
  cvstage_plugins::costmap_to_polygon_list(map, map_to_corners);
  // convert world 2D corners into MiniStage 2D corners
  std::vector< std::vector<cv::Point2i> > polygons;
  polygons.reserve(map.cells.size());
  std::vector<cv::Point2i> one_cell_polygon;
  one_cell_polygon.resize(4);
  for (unsigned int cell_idx = 0; cell_idx < map.cells.size(); ++cell_idx) {
    for (unsigned int corner_idx = 0; corner_idx < 4; ++corner_idx)
      one_cell_polygon[corner_idx] = ms.world2pixel(map_to_corners[4 * cell_idx + corner_idx]);
    // vision_utils::drawPolygon(ms.get_viz(), polygon, true, CV_RGB(0, 0, 255), 3);
    polygons.push_back(one_cell_polygon);
  } // end for (cell_idx)
  // draw polygons
  cv::fillPoly(ms.get_viz(), polygons, inner_color);
  cv::polylines(ms.get_viz(), polygons, true, edge_color, edge_width);
} // end draw_costmap();

} // end namespace cvstage_plugins

#endif // cvstage_PLUGINS_costmap_H
