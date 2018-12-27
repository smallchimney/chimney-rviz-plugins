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
   * File Name     : EditablePoints.h
   * Author        : smallchimney
   * Author Email  : smallchimney@foxmail.com
   * Created Time  : 2018-12-15 11:53:37
   * Last Modified : smallchimney
   * Modified Time : 2018-12-24 18:24:44
************************************************************************* */

#ifndef __RVIZ_PLUGIN_EDITABLE_POINTS_H__
#define __RVIZ_PLUGIN_EDITABLE_POINTS_H__

#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/PolygonStamped.h>

namespace chimney_rviz_plugins {
namespace interactive {

    class EditablePoints {
    protected:
        typedef visualization_msgs::InteractiveMarkerControl Control;
        typedef visualization_msgs::InteractiveMarker InteractiveMarker;
        typedef visualization_msgs::InteractiveMarkerPtr InteractiveMarkerPtr;
        typedef visualization_msgs::Marker Marker;

    public:
        /**
         * @brief  initialize for message handler, service with method only
         * @author smallchimney
         */
        EditablePoints() = default;


        ~EditablePoints() = default;

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
        InteractiveMarkerPtr getEditablePoints(size_t _Idx, float _Diameter, const std_msgs::Header& _Header,
                const std::vector<geometry_msgs::Point32>& _Polygon, const std_msgs::ColorRGBA& _Color);

        static std::string getInteractiveMarkerName(const size_t _PolygonIdx) {
            return std::to_string(_PolygonIdx);
        }

        static size_t getPolygonIdx(const std::string& _Name) {
            return boost::lexical_cast<size_t>(_Name);
        }

        static std::string getControlName(const size_t _VertexIdx) {
            return std::to_string(_VertexIdx);
        }

        static size_t getVertexIdx(const std::string& _Name) {
            return boost::lexical_cast<size_t>(_Name);
        }

    private:

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
        Marker point2Marker(size_t _PolygonSeq, size_t _PointIdx, float _Diameter,
                            const geometry_msgs::Point32& _Point, const std_msgs::ColorRGBA& _Color);

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
        Marker point2TextMarker(size_t _PolygonSeq, size_t _PointIdx, float _Size,
                            const geometry_msgs::Point32& _Point, const std_msgs::ColorRGBA& _Color);

    };

} // namespace interactive
} // namespace chimney_rviz_plugins

#endif //__RVIZ_PLUGIN_EDITABLE_POINTS_H__
