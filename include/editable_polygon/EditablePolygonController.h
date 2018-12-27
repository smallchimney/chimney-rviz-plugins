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
   * File Name     : EditablePolygonController.h
   * Author        : smallchimney
   * Author Email  : smallchimney@foxmail.com
   * Created Time  : 2018-12-17 10:00:00
   * Last Modified : smallchimney
   * Modified Time : 2018-12-26 20:30:17
************************************************************************* */
#ifndef __CHIMNEY_RVIZ_PLUGINS_EDITABLE_POLYGON_CONTROLLER_H__
#define __CHIMNEY_RVIZ_PLUGINS_EDITABLE_POLYGON_CONTROLLER_H__

#include <polygon_painter/PolygonPainter.h>
#include <editable_points/EditablePoints.h>
#include <traits.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <rviz/message_filter_display.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>

namespace chimney_rviz_plugins {
namespace editable_polygon {

    class EditablePolygonController : public rviz::MessageFilterDisplay<geometry_msgs::PolygonStamped> {
    Q_OBJECT
    public:
        /**
         * @brief  pluginlib::ClassLoader creates instances
         *          by calling the default constructor
         * @author smallchimney
         */
        EditablePolygonController();

        /**
         * @brief  free the local point of controller
         * @author smallchimney
         */
        ~EditablePolygonController() override;

    public:

        /**
         * @brief  initialize the local fields except the RViz bond setting
         * @author smallchimney
         */
        void onInitialize() override;

        /**
         * @brief  clean all stamped polygon marker, clean the Ogre scenes' objects
         * @author smallchimney
         */
        void reset() override;

    protected:
        typedef interactive_markers::InteractiveMarkerServer Server;
        typedef visualization_msgs::InteractiveMarkerFeedback Feedback;
        typedef interactive_markers::MenuHandler MenuHandler;

    private:
        ros::NodeHandle m_cNh;

        ros::Publisher m_cPolygonPub;

        /** generate Ogre scene and object according with latest polygon, and display to the screen */
        Type<visualize::PolygonPainter>::Ptr m_pPolygonPainter;

        /** generate interactive marker for new coming polygon, wait for GUI edit result */
        Type<interactive::EditablePoints>::Ptr m_pInteractivePolygon;

        /** connected with RViz */
        Type<Server>::Ptr m_pServer;

        /** connected with RViz */
        Type<MenuHandler>::Ptr m_pCurrentMenuHandler;
        Type<MenuHandler>::Ptr m_pFixedMenuHandler;

        /** store the latest data of polygons */
        Type<geometry_msgs::PolygonStamped>::PtrMap m_mPolygons{};

        Type<rviz::ColorProperty>::Ptr m_pVertexColor, m_pPolygonColor;
        Type<rviz::FloatProperty>::Ptr m_pPolygonAlpha, m_pVertexSize;
        Type<rviz::BoolProperty>::Ptr  m_pEnableLighting, m_pDoubleFace;

        size_t m_ulLatestPolygon;

        /**
         * @brief  listen for initial polygon, change this to interactive polygon and
         *          wait for edit from RViz. the edited result will output to another topic
         * @author smallchimney
         */
        void processMessage(const geometry_msgs::PolygonStampedConstPtr& _Msg) override;

        std_msgs::ColorRGBA getVertexColor();

        /**
         * @brief  record vertex move events, update both the polygon data and the visual markers
         * @author smallchimney
         * @param  _Feedback  RViz event feedback message
         */
        friend void reportMove(const Feedback::ConstPtr& _Feedback);

        /**
         * @brief  record mouse key up events, update the visual markers
         * @author smallchimney
         * @param  _Feedback  RViz event feedback message
         */
        friend void reportRelease(const Feedback::ConstPtr& _Feedback);

        /**
         * @brief  record menu select events, publish the polygon
         *         to "/chimney_rviz_plugins/editable_polygon/result"
         * @author smallchimney
         * @param  _Feedback  RViz event feedback message
         */
        friend void publishPolygon(const Feedback::ConstPtr& _Feedback);

        /**
         * @brief  record menu select events, fix the polygon current operating, update
         *         the menu for it, publish the polygon data and wait for the next polygon
         * @author smallchimney
         * @param  _Feedback  RViz event feedback message
         */
        friend void fixPolygon(const Feedback::ConstPtr& _Feedback);

        /**
         * @brief  delete selected polygon from memory and screen
         * @author smallchimney
         * @param  _Feedback  RViz event feedback message
         */
        friend void deletePolygon(const Feedback::ConstPtr& _Feedback);

    private Q_SLOTS:
        void updateAllPolygonColor();
        void updateVertexColor();
        void updateVertexSize();
        void updateEnableLighting();
        void updateDoubleFace();

    };

} // namespace editable_polygon
} // namespace chimney_rviz_plugins

#endif //__CHIMNEY_RVIZ_PLUGINS_EDITABLE_POLYGON_CONTROLLER_H__
