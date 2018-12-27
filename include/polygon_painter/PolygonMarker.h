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
   * File Name     : PolygonMarker.h
   * Author        : smallchimney
   * Author Email  : smallchimney@foxmail.com
   * Created Time  : 2018-12-24 21:03:12
   * Last Modified : smallchimney
   * Modified Time : 2018-12-26 19:05:24
************************************************************************* */
#ifndef __CHIMNEY_RVIZ_PLUGINS_POLYGON_MARKER_H__
#define __CHIMNEY_RVIZ_PLUGINS_POLYGON_MARKER_H__

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Polygon.h>

namespace chimney_rviz_plugins {
namespace visualize {

    class PolygonMarker {
    protected:
        typedef visualization_msgs::Marker Marker;

    public:
        PolygonMarker() = delete;
        virtual ~PolygonMarker() = delete;

        /**
         * @brief  generate a polygon border marker
         * @author smallchimney
         * @param  _Id      the unique ID for the marker in the topic
         * @param  _Header  the header of the marker, which is mark the frame that the pose belong to.
         * @param  _Polygon the polygon data
         * @param  _Color   the color of the marker
         * @param  _Size    the size of border line in meter
         * @return          the generated marker
         */
        static Marker getBorder(int _Id, const std_msgs::Header& _Header,
                const geometry_msgs::PolygonConstPtr& _Polygon,
                const std_msgs::ColorRGBA& _Color, float _Size);

        /**
         * @brief  generate a polygon border marker for interactive, the header field will be empty which
         *         means that relative to the pose of the parent interactive marker.
         * @author smallchimney
         * @param  _Id      the unique ID for the marker in the topic
         * @param  _Polygon the polygon data
         * @param  _Color   the color of the marker
         * @param  _Size    the size of border line in meter
         * @return          the generated marker
         */
        static Marker getBorder(int _Id, const geometry_msgs::PolygonConstPtr& _Polygon,
                const std_msgs::ColorRGBA& _Color, float _Size);

        /**
         * @brief  generate a polygon area marker
         * @author smallchimney
         * @param  _Id      the unique ID for the marker in the topic
         * @param  _Header  the header of the marker, which is mark the frame that the pose belong to.
         * @param  _Polygon the polygon data
         * @param  _Color   the color of the marker
         * @param  _Size    the size of border line in meter
         * @return          the generated marker
         */
        static Marker getArea(int _Id, const std_msgs::Header& _Header,
                                const geometry_msgs::PolygonConstPtr& _Polygon,
                                const std_msgs::ColorRGBA& _Color, float _Size);


        /**
         * @brief  generate a polygon area marker for interactive, the header field will be empty which
         *         means that relative to the pose of the parent interactive marker.
         * @author smallchimney
         * @param  _Id      the unique ID for the marker in the topic
         * @param  _Polygon the polygon data
         * @param  _Color   the color of the marker
         * @param  _Size    the size of border line in meter
         * @return          the generated marker
         */
        static Marker getArea(int _Id, const geometry_msgs::PolygonConstPtr& _Polygon,
                                const std_msgs::ColorRGBA& _Color, float _Size);

    private:
        static geometry_msgs::Point transform(const geometry_msgs::Point32& _Point) {
            geometry_msgs::Point point;
            point.x = _Point.x;
            point.y = _Point.y;
            point.z = _Point.z;
            return point;
        }
    };

} // namespace visualize
} // namespace chimney_rviz_plugins

#endif //__CHIMNEY_RVIZ_PLUGINS_POLYGON_MARKER_H__
