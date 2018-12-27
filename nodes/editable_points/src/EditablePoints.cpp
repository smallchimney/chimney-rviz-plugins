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
   * File Name     : EditablePoints.cpp
   * Author        : smallchimney
   * Author Email  : smallchimney@foxmail.com
   * Created Time  : 2018-12-15 11:23:30
   * Last Modified : smallchimney
   * Modified Time : 2018-12-24 18:25:59
************************************************************************* */
#include <editable_points/EditablePoints.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace chimney_rviz_plugins {
namespace interactive {

static geometry_msgs::Quaternion defaultOrientation();

/* ********************************************************************************
 *                              SERVICE CALLBACKS                                 *
 ******************************************************************************** */

/**
 * @brief  generate interactive marker for specific polygon to edit
 * @author smallchimney
 * @param  _Idx         the index of polygon, should be global unique
 * @param  _Diameter    the diameter of the marker that control the polygon in meter
 * @param  _Header      the header stand for fixed frame and data timestamp
 * @param  _Polygon     the original input polygon
 * @param  _Color       the color RGB-A of the marker
 * @return              the point of generated interactive marker's vertex color
 */
EditablePoints::InteractiveMarkerPtr EditablePoints::getEditablePoints(
        const size_t _Idx, const float _Diameter, const std_msgs::Header& _Header,
        const std::vector<geometry_msgs::Point32>& _Polygon, const std_msgs::ColorRGBA& _Color) {
    InteractiveMarkerPtr marker(new InteractiveMarker);
    marker -> scale = 1;
    marker -> name = getInteractiveMarkerName(_Idx);
    marker -> description = "edit polygon shape: move vertex";
    marker -> header = _Header;

    // since we control the point, this is meaningless :)
    const static geometry_msgs::Quaternion orientation = defaultOrientation();
    marker -> pose.orientation = orientation;
    tf2::toMsg(tf2::Vector3(0., 0., 0.), marker -> pose.position);

    size_t i = 0;
    for (const auto& point : _Polygon) {
        Control control;
        control.name = getControlName(i);
        control.description = "Move the vertex " + control.name;

        control.interaction_mode = Control::MOVE_PLANE;
        control.always_visible = static_cast<unsigned char>(false);

        // since we control the point, this is meaningless :)
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;

        control.markers.emplace_back(point2Marker(
                _Idx, i, _Diameter, point, _Color));
        control.markers.emplace_back(point2TextMarker(
                _Idx, i++, _Diameter / 3.f, point, _Color));
        marker -> controls.emplace_back(control);
    }

    return marker;
}

/* ********************************************************************************
 *                              FUNCTIONAL METHODS                                *
 ******************************************************************************** */

/**
 * @brief  generate visualize marker for specific point to edit
 * @author smallchimney
 * @param  _PolygonSeq  the sequence of the polygon defined in header
 * @param  _PointIdx    the point index inside the polygon
 * @param  _Diameter    the diameter of the marker that control the polygon in meter
 * @param  _Point       the center point position of the marker
 * @param  _Color       the color RGB-A of the marker
 * @return              the generated marker
 */
EditablePoints::Marker EditablePoints::point2Marker(const size_t _PolygonSeq, size_t _PointIdx,
        const float _Diameter, const geometry_msgs::Point32& _Point, const std_msgs::ColorRGBA& _Color) {
    Marker point;
    // mark the marker ID as unique and found easily.
    std::size_t hash = 0;
    boost::hash_combine(hash, _PolygonSeq);
    boost::hash_combine(hash, _PointIdx);
    point.id = static_cast<int>(hash);
    point.ns = "chimney_rviz_plugins/editable_polygon";
    point.action = Marker::ADD;
    point.type = Marker::SPHERE;
    // we paint a circle to edit the vertex
    point.scale = tf2::toMsg(tf2::Vector3(_Diameter, _Diameter, .01));
    // this could be changed if needed
    point.frame_locked = static_cast<unsigned char>(true);

    point.pose.position.x = _Point.x;
    point.pose.position.y = _Point.y;
    point.pose.position.z = _Point.z;
    point.color.r = 1.f - _Color.r;
    point.color.g = 1.f - _Color.g;
    point.color.b = 1.f - _Color.b;
    point.color.a = _Color.a;

    // since we control the point, this is meaningless :)
    const static geometry_msgs::Quaternion orientation = defaultOrientation();
    point.pose.orientation = orientation;
    point.color = _Color;

    return point;
}

/**
 * @brief  generate visualize marker for specific point to show text label
 * @author smallchimney
 * @param  _PolygonSeq  the sequence of the polygon defined in header
 * @param  _Size        the font size
 * @param  _Diameter    the diameter of the marker that control the polygon in meter
 * @param  _Point       the center point position of the marker
 * @param  _Color       the color RGB-A of the marker
 * @return              the generated marker
 */
EditablePoints::Marker EditablePoints::point2TextMarker(const size_t _PolygonSeq, size_t _PointIdx,
        const float _Size, const geometry_msgs::Point32& _Point, const std_msgs::ColorRGBA& _Color) {
    Marker point;
    // mark the marker ID as unique and found easily.
    std::size_t hash = 0;
    boost::hash_combine(hash, _PolygonSeq);
    boost::hash_combine(hash, _PointIdx);
    boost::hash_combine(hash, "text");
    point.id = static_cast<int>(hash);
    point.ns = "chimney_rviz_plugins/editable_polygon";
    point.action = Marker::ADD;
    point.type = Marker::TEXT_VIEW_FACING;
    point.scale.z = _Size;    // font size
    std::stringstream text;
    text << "P" << _PolygonSeq << "::V" << _PointIdx + 1;
    point.text = text.str();
    // this could be changed if needed
    point.frame_locked = static_cast<unsigned char>(true);

    point.pose.position.x = _Point.x;
    point.pose.position.y = _Point.y;
    point.pose.position.z = _Point.z;
    point.color.r = 1.f - _Color.r;
    point.color.g = 1.f - _Color.g;
    point.color.b = 1.f - _Color.b;
    point.color.a = 1.f;

    // since we control the point, this is meaningless :)
    const static geometry_msgs::Quaternion orientation = defaultOrientation();
    point.pose.orientation = orientation;

    return point;
}

/* ********************************************************************************
 *                                LOCAL METHODS                                   *
 ******************************************************************************** */

/**
 * @brief  generate a default orientation for setting, this method could not be used?
 * @author smallchimney
 * @return default orientation that faced to X-Axis
 */
static geometry_msgs::Quaternion defaultOrientation() {
    tf2::Quaternion quaternion;
    quaternion.setRPY(0., 0., 0.);
    return tf2::toMsg(quaternion);
}

} // namespace interactive
} // namespace chimney_rviz_plugins
