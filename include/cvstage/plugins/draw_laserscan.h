/*!
  \file        draw_laserscan.h
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

#ifndef CVSTAGE_PLUGINS_COSTMAP_H
#define CVSTAGE_PLUGINS_COSTMAP_H

#include "cvstage/cvstage.h"
#include <sensor_msgs/LaserScan.h>

namespace cvstage_plugins {

void draw_scan(MiniStage _ms, const sensor_msgs::LaserScanConstPtr & scan,
               int radius, const cv::Scalar& scan_color, int thickness = 1,
               int lineType = cv::LINE_8, int shift = 0) {
  unsigned int npts = scan->ranges.size();
  for (unsigned int i = 0; i < npts; ++i) {
    // conversion polar -> Cartesian coordinates
    double theta = scan->angle_min + scan->angle_increment * i;
    double x = scan->ranges[i] * cos(theta);
    double y = scan->ranges[i] * sin(theta);
    cv::circle(_ms.get_viz(), _ms.world2pixel(x, y), radius, scan_color, thickness,
               lineType, shift);
  } // end for i
}


} // end namespace cvstage_plugins

#endif // CVSTAGE_PLUGINS_COSTMAP_H
