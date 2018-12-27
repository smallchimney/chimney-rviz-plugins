/**************************************************************************
 * Copyright (c) 2018 Chimney Xu. All Rights Reserve.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 **************************************************************************/
/* *************************************************************************
   * File Name     : PolygonUtils.h
   * Author        : smallchimney
   * Author Email  : smallchimney@foxmail.com
   * Created Time  : 2018-12-25 14:17:07
   * Last Modified : smallchimney
   * Modified Time : 2018-12-25 14:40:47
************************************************************************* */
#ifndef __CHIMNEY_RVIZ_PLUGINS_POLYGON_UTILS_H__
#define __CHIMNEY_RVIZ_PLUGINS_POLYGON_UTILS_H__

#include <traits.h>
#include <geometry_msgs/Polygon.h>

#include <pcl_conversions/pcl_conversions.h>

namespace chimney_rviz_plugins {
namespace util {

    class PolygonUtils {
    public:
        typedef boost::array<geometry_msgs::Point32, 3> Triangle;
        typedef std::vector<Triangle>                   TriangleList;
        typedef Type<TriangleList>::Ptr                 TriangleListPtr;

    public:
        PolygonUtils() = delete;
        virtual ~PolygonUtils() = delete;

        /**
          * @brief  decompose polygon into triangles array for visualize
          * @author smallchimney
          * @param  _Polygon  the original polygon in 3D
          * @return           the array of triangles decomposed from polygon
          */
        static TriangleListPtr decomposeToTriangles(
                const geometry_msgs::Polygon& _Polygon) noexcept(false);

    private:
        static geometry_msgs::Point32 transform(const pcl::PointXYZ& _Point)  {
            geometry_msgs::Point32 point;
            point.x = _Point.x;
            point.y = _Point.y;
            point.z = _Point.z;
            return point;
        }

    };

} // namespace util
} // namespace chimney_rviz_plugins

#endif //__CHIMNEY_RVIZ_PLUGINS_POLYGON_UTILS_H__
