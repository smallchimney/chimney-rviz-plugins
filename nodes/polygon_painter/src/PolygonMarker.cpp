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
   * File Name     : PolygonMarker.cpp
   * Author        : smallchimney
   * Author Email  : smallchimney@foxmail.com
   * Created Time  : 2018-12-24 20:58:31
   * Last Modified : smallchimney
   * Modified Time : 2018-12-24 20:58:31
************************************************************************* */
#include <polygon_painter/PolygonMarker.h>
#include <utils/polygon/PolygonUtils.h>

#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace chimney_rviz_plugins {
namespace visualize {

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
PolygonMarker::Marker PolygonMarker::getBorder(int _Id,
        const std_msgs::Header& _Header, const geometry_msgs::PolygonConstPtr& _Polygon,
        const std_msgs::ColorRGBA& _Color, float _Size) {
    Marker marker = getBorder(_Id, _Polygon, _Color, _Size);
    marker.header = _Header;
    return marker;
}

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
PolygonMarker::Marker PolygonMarker::getBorder(int _Id,
        const geometry_msgs::PolygonConstPtr& _Polygon,
        const std_msgs::ColorRGBA& _Color, const float _Size) {
    Marker marker;
    marker.id = _Id;
    marker.action = Marker::ADD;
    marker.type = Marker::LINE_LIST;
    marker.scale = tf2::toMsg(tf2::Vector3(_Size, _Size, .01));
    marker.color = _Color;

    std::vector<geometry_msgs::Point> polygon;
    for(const auto& point : _Polygon -> points) {
        polygon.emplace_back(transform(point));
    }

    for(size_t i = 1; i < polygon.size(); i++) {
        marker.points.emplace_back(polygon[i - 1]);
        marker.points.emplace_back(polygon[i]);
    }
    marker.points.emplace_back(polygon[polygon.size() - 1]);
    marker.points.emplace_back(polygon[0]);
    return marker;
}

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
PolygonMarker::Marker PolygonMarker::getArea(int _Id,
        const std_msgs::Header& _Header, const geometry_msgs::PolygonConstPtr& _Polygon,
        const std_msgs::ColorRGBA& _Color, float _Size) {
    Marker marker = getArea(_Id, _Polygon, _Color, _Size);
    marker.header = _Header;
    return marker;
}

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
PolygonMarker::Marker PolygonMarker::getArea(int _Id,
        const geometry_msgs::PolygonConstPtr& _Polygon,
        const std_msgs::ColorRGBA& _Color, const float _Size) {
    Marker marker;
    marker.id = _Id;
    marker.action = Marker::ADD;
    marker.type = Marker::LINE_LIST;
    // LINE_LIST markers use only the x component of scale, for the line width
    marker.scale.x = _Size;
    marker.color = _Color;

    for(const auto& triangle : *util::PolygonUtils::decomposeToTriangles(*_Polygon)) {
        for(const auto& point : triangle)
            marker.points.emplace_back(transform(point));
    }
    return marker;
}


} // namespace visualize
} // namespace chimney_rviz_plugins
